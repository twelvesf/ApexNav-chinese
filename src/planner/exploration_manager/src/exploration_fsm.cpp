
/**
 * @file exploration_fsm.cpp
 * @brief 仿真模式探索有限状态机实现
 * @author ApexNav Team
 *
 * 该文件实现了ExplorationFSM类，用于Habitat仿真环境中的自主探索任务。
 * 状态机管理从初始化到任务完成的整个探索过程，包括动作规划、卡死逃脱、
 * 前沿探索和对象搜索等功能。
 */

#include <exploration_manager/exploration_manager.h>
#include <exploration_manager/exploration_fsm.h>
#include <exploration_manager/exploration_data.h>
#include <vis_utils/planning_visualization.h>

namespace apexnav_planner {

/**
 * @brief 初始化探索有限状态机
 * @param nh ROS节点句柄
 *
 * 设置FSM参数、初始化各模块、建立ROS通信接口
 */
void ExplorationFSM::init(ros::NodeHandle& nh)
{
  nh_ = nh;
  fp_.reset(new FSMParam);           // FSM参数
  fd_.reset(new FSMData);            // FSM数据

  /* 初始化主要模块 */
  expl_manager_.reset(new ExplorationManager);    // 探索管理器
  expl_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));  // 可视化工具
  fp_->vis_scale_ = expl_manager_->sdf_map_->getResolution() * FSMConstants::VIS_SCALE_FACTOR;

  state_ = ROS_STATE::INIT;  // 初始状态

  /* ROS定时器 */
  exec_timer_ = nh.createTimer(
      ros::Duration(FSMConstants::EXEC_TIMER_DURATION), &ExplorationFSM::FSMCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(FSMConstants::FRONTIER_TIMER_DURATION),
      &ExplorationFSM::frontierCallback, this);

  /* ROS订阅者 - 接收外部信息 */
  trigger_sub_ = nh.subscribe("/move_base_simple/goal", 10, &ExplorationFSM::triggerCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 10, &ExplorationFSM::odometryCallback, this);
  habitat_state_sub_ =
      nh.subscribe("/habitat/state", 10, &ExplorationFSM::habitatStateCallback, this);
  confidence_threshold_sub_ = node_.subscribe(
      "/detector/confidence_threshold", 10, &ExplorationFSM::confidenceThresholdCallback, this);

  /* ROS发布者 - 发送控制命令和状态信息 */
  ros_state_pub_ = nh.advertise<std_msgs::Int32>("/ros/state", 10);
  expl_state_pub_ = nh.advertise<std_msgs::Int32>("/ros/expl_state", 10);
  action_pub_ = nh.advertise<std_msgs::Int32>("/habitat/plan_action", 10);
  expl_result_pub_ = nh.advertise<std_msgs::Int32>("/ros/expl_result", 10);
  robot_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/robot", 10);
}

/**
 * @brief 有限状态机主回调函数
 * @param e 定时器事件
 *
 * 这是FSM的核心循环，根据当前状态执行相应的逻辑：
 * INIT → WAIT_TRIGGER → PLAN_ACTION → PUB_ACTION → WAIT_ACTION_FINISH → FINISH
 */
void ExplorationFSM::FSMCallback(const ros::TimerEvent& e)
{
  exec_timer_.stop();  // 停止定时器，避免重复执行

  // 发布当前状态
  std_msgs::Int32 ros_state_msg;
  ros_state_msg.data = state_;
  ros_state_pub_.publish(ros_state_msg);

  // 状态机主循环
  switch (state_) {
    case ROS_STATE::INIT: {
      // 等待里程计和置信度阈值准备就绪
      if (!fd_->have_odom_ || !fd_->have_confidence_) {
        ROS_WARN_THROTTLE(1.0, "No odom || No target confidence threshold.");
        exec_timer_.start();
        return;
      }
      // 前置条件满足，进入等待触发状态
      clearVisMarker();
      transitState(ROS_STATE::WAIT_TRIGGER, "FSM");
      break;
    }

    case ROS_STATE::WAIT_TRIGGER: {
      // 等待外部触发信号启动探索
      ROS_WARN_THROTTLE(1.0, "Wait for trigger.");
      break;
    }

    case ROS_STATE::FINISH: {
      // 任务完成状态
      if (!fd_->have_finished_) {
        fd_->have_finished_ = true;
        clearVisMarker();
        // 发送停止动作
        std_msgs::Int32 action_msg;
        action_msg.data = ACTION::STOP;
        action_pub_.publish(action_msg);
      }
      ROS_WARN_THROTTLE(1.0, "Finish One Episode!!!");
      break;
    }

    case ROS_STATE::PLAN_ACTION: {
      // 动作规划状态
      // 初始校准序列：执行方向校准转弯动作
      if (fd_->init_action_count_ < 1 + 12 + 1 + 12) {
        // 执行预定义的初始动作序列（总共26个动作）
        if (fd_->init_action_count_ < 1)
          fd_->newest_action_ = ACTION::TURN_DOWN;      // 向下转
        else if (fd_->init_action_count_ < 1 + 12)
          fd_->newest_action_ = ACTION::TURN_LEFT;      // 左转12次
        else if (fd_->init_action_count_ < 1 + 12 + 1)
          fd_->newest_action_ = ACTION::TURN_UP;        // 向上转
        else
          fd_->newest_action_ = ACTION::TURN_LEFT;      // 继续左转12次

        ROS_WARN("Init Mode Process -----> (%d/26)", fd_->init_action_count_);
        fd_->init_action_count_++;
        transitState(ROS_STATE::PUB_ACTION, "FSM");
        updateFrontierAndObject();
      }
      else {
        // 主要规划阶段：确定机器人姿态并调用动作规划器
        fd_->start_pt_ = fd_->odom_pos_;           // 当前位置
        fd_->start_yaw_(0) = fd_->odom_yaw_;       // 当前朝向

        // 调用动作规划器
        auto t1 = ros::Time::now();
        fd_->final_result_ = callActionPlanner();
        double call_action_planner_time = (ros::Time::now() - t1).toSec();
        ROS_INFO_THROTTLE(
            10.0, "[Calculating Time] Planning process time = %.3f s", call_action_planner_time);

        // 发布探索状态
        std_msgs::Int32 expl_state_msg;
        expl_state_msg.data = fd_->final_result_;
        expl_state_pub_.publish(expl_state_msg);

        // 根据规划结果决定下一状态
        if (fd_->final_result_ == FINAL_RESULT::EXPLORE ||
            fd_->final_result_ == FINAL_RESULT::SEARCH_OBJECT)
          transitState(ROS_STATE::PUB_ACTION, "FSM");      // 继续执行动作
        else
          transitState(ROS_STATE::FINISH, "FSM");          // 任务完成
      }
      visualize();  // 可视化当前状态
      break;
    }

    case ROS_STATE::PUB_ACTION: {
      // 发布动作状态
      std_msgs::Int32 action_msg;
      action_msg.data = fd_->newest_action_;
      action_pub_.publish(action_msg);
      transitState(ROS_STATE::WAIT_ACTION_FINISH, "FSM");
      break;
    }

    case ROS_STATE::WAIT_ACTION_FINISH: {
      // 等待动作执行完成
      exec_timer_.start();
      break;
    }
  }
  exec_timer_.start();  // 重新启动定时器
}

/**
 * @brief 规划下一个动作（核心规划函数）
 * @return 最终结果，指示规划的动作类型和探索状态
 *
 * 这是整个系统的核心规划函数，决定机器人下一步应该采取什么动作。
 * 处理障碍物 avoidance、前沿探索、对象搜索和卡死恢复等情况。
 */
int ExplorationFSM::callActionPlanner()
{
  // 定义各种距离阈值常量
  const double stucking_distance = FSMConstants::STUCKING_DISTANCE;      // 卡死距离阈值
  const double reach_distance = FSMConstants::REACH_DISTANCE;            // 到达目标距离
  const double soft_reach_distance = FSMConstants::SOFT_REACH_DISTANCE;  // 软到达距离

  // 更新前沿和对象信息，返回是否有前沿变化
  bool frontier_change_flag = updateFrontierAndObject();

  int expl_res, final_res;  // 探索结果和最终结果

  // 获取当前位置和上次位置
  Eigen::Vector2d current_pos = Eigen::Vector2d(fd_->start_pt_(0), fd_->start_pt_(1));
  Eigen::Vector2d last_pos = Eigen::Vector2d(fd_->last_start_pos_(0), fd_->last_start_pos_(1));
  double current_yaw = fd_->start_yaw_(0);
  fd_->last_start_pos_ = fd_->start_pt_;  // 更新上次位置

  // ===== 检查是否到达目标对象 =====
  if (fd_->final_result_ == FINAL_RESULT::SEARCH_OBJECT &&
      (current_pos - expl_manager_->ed_->next_pos_).norm() < reach_distance) {
    ROS_ERROR("Reach the object successfully!!!");
    final_res = FINAL_RESULT::REACH_OBJECT;
    return final_res;
  }

  /******* 卡死逃脱逻辑开始 *******/
  // 检测机器人是否卡住并启动逃脱序列
  int last_action = fd_->newest_action_;
  if (!fd_->escape_stucking_flag_ && (current_pos - last_pos).norm() < stucking_distance &&
      last_action == ACTION::MOVE_FORWARD) {

    // 再次检查软到达条件
    if (fd_->final_result_ == FINAL_RESULT::SEARCH_OBJECT &&
        (current_pos - expl_manager_->ed_->next_pos_).norm() < soft_reach_distance) {
      ROS_ERROR("Reach the object successfully!!!");
      final_res = FINAL_RESULT::REACH_OBJECT;
      return final_res;
    }

    // 检查是否在已知的卡死点附近
    bool past_stucking_flag = false;
    for (auto stucking_point : fd_->stucking_points_) {
      Vector2d stucking_pos = Vector2d(stucking_point(0), stucking_point(1));
      double stucking_yaw = stucking_point(2);
      if ((stucking_pos - current_pos).norm() < stucking_distance &&
          fabs(stucking_yaw - current_yaw) < FSMConstants::ACTION_ANGLE) {
        past_stucking_flag = true;
        ROS_ERROR("Still stuck at the same place");
        break;
      }
    }

    // 如果不是已知卡死点，启动逃脱模式
    if (!past_stucking_flag) {
      fd_->escape_stucking_flag_ = true;
      fd_->escape_stucking_count_ = 0;
      fd_->escape_stucking_pos_ = current_pos;
      fd_->escape_stucking_yaw_ = current_yaw;
    }
  }

  // 检查是否已经逃脱成功
  if (fd_->escape_stucking_flag_ && (current_pos - last_pos).norm() >= stucking_distance) {
    ROS_ERROR("Escaped from stuck state.");
    fd_->escape_stucking_flag_ = false;
  }

  // 执行逃脱动作序列
  if (fd_->escape_stucking_flag_) {
    ROS_ERROR("Escaping stuck...");
    // 预定义的逃脱动作序列：右转-前进-右转-前进-左转x3-前进-左转-前进
    if (fd_->escape_stucking_count_ == 0)
      fd_->newest_action_ = ACTION::TURN_RIGHT;
    else if (fd_->escape_stucking_count_ == 1)
      fd_->newest_action_ = ACTION::MOVE_FORWARD;
    else if (fd_->escape_stucking_count_ == 2)
      fd_->newest_action_ = ACTION::TURN_RIGHT;
    else if (fd_->escape_stucking_count_ == 3)
      fd_->newest_action_ = ACTION::MOVE_FORWARD;
    else if (fd_->escape_stucking_count_ == 4)
      fd_->newest_action_ = ACTION::TURN_LEFT;
    else if (fd_->escape_stucking_count_ == 5)
      fd_->newest_action_ = ACTION::TURN_LEFT;
    else if (fd_->escape_stucking_count_ == 6)
      fd_->newest_action_ = ACTION::TURN_LEFT;
    else if (fd_->escape_stucking_count_ == 7)
      fd_->newest_action_ = ACTION::MOVE_FORWARD;
    else if (fd_->escape_stucking_count_ == 8)
      fd_->newest_action_ = ACTION::TURN_LEFT;
    else if (fd_->escape_stucking_count_ == 9)
      fd_->newest_action_ = ACTION::MOVE_FORWARD;
    else {
      // 逃脱失败：标记区域为占用并记录卡死点
      ROS_ERROR("Cannot escape stuck state.");
      fd_->escape_stucking_flag_ = false;

      // 在当前位置和前方位置标记为占用
      expl_manager_->sdf_map_->setForceOccGrid(current_pos);
      double forward_distance = FSMConstants::FORWARD_DISTANCE;
      Eigen::Vector2d forward_pos = fd_->escape_stucking_pos_;
      forward_pos(0) += forward_distance * cos(fd_->escape_stucking_yaw_);
      forward_pos(1) += forward_distance * sin(fd_->escape_stucking_yaw_);
      expl_manager_->sdf_map_->setForceOccGrid(forward_pos);

      // 在更远的前方位置也标记为占用
      forward_distance = FSMConstants::FORWARD_DISTANCE * 2.0;
      forward_pos = fd_->escape_stucking_pos_;
      forward_pos(0) += forward_distance * cos(fd_->escape_stucking_yaw_);
      forward_pos(1) += forward_distance * sin(fd_->escape_stucking_yaw_);
      expl_manager_->sdf_map_->setForceOccGrid(forward_pos);

      // 设置前沿为休眠状态并记录卡死点
      fd_->dormant_frontier_flag_ = true;
      Vector3d stucking_point(
          fd_->escape_stucking_pos_(0), fd_->escape_stucking_pos_(1), fd_->escape_stucking_yaw_);
      fd_->stucking_points_.push_back(stucking_point);
    }

    if (fd_->escape_stucking_flag_) {
      fd_->escape_stucking_count_++;
      return fd_->final_result_;  // 返回之前的最终结果
    }
  }

  /******* 决定是否重新规划路径（稳定性启发式）开始 *******/
  // 使用路径稳定性来减少在不同前沿目标之间的振荡
  vector<Vector2d> last_next_best_path = expl_manager_->ed_->next_best_path_;
  Vector2d last_next_pos = expl_manager_->ed_->next_pos_;

  // 如果设置了休眠前沿标志，强制重新规划
  if (fd_->dormant_frontier_flag_) {
    fd_->replan_flag_ = true;
    fd_->dormant_frontier_flag_ = false;
  }
  else if (fd_->final_result_ == FINAL_RESULT::EXPLORE && !frontier_change_flag)
    fd_->replan_flag_ = false;  // 前沿没有变化且在探索模式，不重新规划

  // 调用探索管理器规划下一最佳点
  expl_res = expl_manager_->planNextBestPoint(fd_->start_pt_, fd_->start_yaw_(0));

  // 根据探索结果决定是否需要重新规划
  if (expl_res != EXPL_RESULT::EXPLORATION) {
    fd_->replan_flag_ = true;  // 非探索结果都需要重新规划
  }
  if (expl_res == EXPL_RESULT::EXPLORATION && !fd_->replan_flag_) {
    // 恢复之前的路径和位置（保持稳定性）
    expl_manager_->ed_->next_best_path_ = last_next_best_path;
    expl_manager_->ed_->next_pos_ = last_next_pos;
    fd_->replan_flag_ = true;  // 仍然标记为已重新规划
  }
  /******* 决定是否重新规划路径（稳定性启发式）结束 *******/

  // 发布探索结果供监控
  std_msgs::Int32 expl_result_msg;
  expl_result_msg.data = expl_res;
  expl_result_pub_.publish(expl_result_msg);

  // 根据探索结果确定当前高层状态
  if (expl_res == EXPL_RESULT::EXPLORATION)
    final_res = FINAL_RESULT::EXPLORE;              // 继续探索
  else if (expl_res == EXPL_RESULT::NO_COVERABLE_FRONTIER ||
           expl_res == EXPL_RESULT::NO_PASSABLE_FRONTIER)
    final_res = FINAL_RESULT::NO_FRONTIER;          // 没有可用的前沿
  else
    final_res = FINAL_RESULT::SEARCH_OBJECT;        // 搜索对象

  // 检查是否有有效的路径
  if (final_res == FINAL_RESULT::NO_FRONTIER || expl_manager_->ed_->next_best_path_.empty()) {
    ROS_WARN("No (passable) frontier");
    return final_res;
  }

  // 获取目标位置并计算距离
  Eigen::Vector2d end_pos = expl_manager_->ed_->next_pos_;
  Eigen::Vector2d last_end_pos = fd_->last_next_pos_;
  fd_->last_next_pos_ = end_pos;
  double min_dist = (current_pos - end_pos).norm();
  ROS_WARN("To the next point (%.2fm %.2fm), distance = %.2f m", end_pos(0), end_pos(1), min_dist);

  // 处理在向特定前沿探索时卡住的情况
  if (final_res == FINAL_RESULT::EXPLORE) {
    // 如果非常接近目标但仍在探索，强制设置前沿为休眠
    if (min_dist < FSMConstants::FORCE_DORMANT_DISTANCE) {
      ROS_ERROR("Force set dormant frontier.");
      expl_manager_->frontier_map2d_->setForceDormantFrontier(end_pos);
      fd_->dormant_frontier_flag_ = true;
    }

    // 计算连续次数与相同目标位置同时卡住
    if ((end_pos - last_end_pos).norm() < 1e-3 &&
        (current_pos - last_pos).norm() < stucking_distance) {
      fd_->stucking_next_pos_count_++;
      ROS_ERROR_COND(fd_->stucking_next_pos_count_ > 8, "stucking_next_pos_count_ = %d",
          fd_->stucking_next_pos_count_);
    }
    else
      fd_->stucking_next_pos_count_ = 0;

    // 如果卡住次数过多，设置前沿为休眠
    if (fd_->stucking_next_pos_count_ >= FSMConstants::MAX_STUCKING_NEXT_POS_COUNT) {
      ROS_ERROR("Set dormant frontier.");
      fd_->stucking_action_count_ = 0;
      fd_->stucking_next_pos_count_ = 0;
      expl_manager_->frontier_map2d_->setForceDormantFrontier(end_pos);
      fd_->dormant_frontier_flag_ = true;
    }
  }

  // 全局跟踪连续卡死动作次数
  if ((current_pos - last_pos).norm() < stucking_distance) {
    fd_->stucking_action_count_++;
    ROS_ERROR_COND(fd_->stucking_action_count_ > 15, "Stucking action count = %d",
        fd_->stucking_action_count_);
  }
  else
    fd_->stucking_action_count_ = 0;

  // 如果全局卡死时间过长，终止episode
  if (fd_->stucking_action_count_ >= FSMConstants::MAX_STUCKING_COUNT) {
    ROS_ERROR("Stuck for too long, stopping episode.");
    final_res = FINAL_RESULT::STUCKING;
    return final_res;
  }

  // 根据探索结果规划具体动作
  if (expl_res == EXPL_RESULT::SEARCH_EXTREME)
    // 极端搜索模式，不需要安全检查
    fd_->newest_action_ =
        planNextBestAction(current_pos, current_yaw, expl_manager_->ed_->next_best_path_, false);
  else
    // 正常模式，需要安全检查
    fd_->newest_action_ =
        planNextBestAction(current_pos, current_yaw, expl_manager_->ed_->next_best_path_);

  return final_res;
}

/**
 * @brief 规划下一最佳动作
 * @param current_pos 当前机器人位置
 * @param current_yaw 当前机器人朝向
 * @param path 规划的路径点序列
 * @param need_safety 是否需要安全检查
 * @return 下一动作类型
 *
 * 根据当前位置、朝向和目标路径，决定下一步应该执行的动作（前进或转弯）
 */
int ExplorationFSM::planNextBestAction(
    Vector2d current_pos, double current_yaw, const vector<Vector2d>& path, bool need_safety)
{
  const double local_distance = FSMConstants::LOCAL_DISTANCE;  // 局部目标距离

  // 根据路径和局部距离更新目标位置
  Vector2d local_pos = selectLocalTarget(current_pos, path, local_distance);
  fd_->local_pos_ = local_pos;

  // 计算最佳步骤，考虑障碍物和安全距离
  Vector2d best_step;
  if ((current_pos - path.back()).norm() > FSMConstants::ACTION_DISTANCE && need_safety)
    // 距离目标较远且需要安全检查时，计算最佳步骤
    best_step = computeBestStep(current_pos, current_yaw, local_pos);
  else
    // 否则直接使用局部目标位置
    best_step = local_pos;

  // 根据最佳步骤方向计算目标朝向
  double target_yaw = std::atan2(best_step(1) - current_pos(1), best_step(0) - current_pos(0));

  // 根据当前朝向和目标朝向决定是转弯还是前进
  return decideNextAction(current_yaw, target_yaw);
}

/**
 * @brief 选择局部目标位置
 * @param current_pos 当前位置
 * @param path 完整路径
 * @param local_distance 局部距离限制
 * @return 局部目标位置
 *
 * 从路径中选择一个在指定局部距离内的目标点，用于短期导航
 */
Vector2d ExplorationFSM::selectLocalTarget(
    const Vector2d& current_pos, const vector<Vector2d>& path, const double& local_distance)
{
  Vector2d target_pos = path.back();  // 默认使用路径终点

  // 找到路径中距离当前位置最近的点作为搜索起点
  int start_path_id = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < (int)path.size() - 1; i++) {
    Eigen::Vector2d pos = path[i];
    if ((pos - current_pos).norm() < min_dist) {
      min_dist = (pos - current_pos).norm();
      start_path_id = i + 1;
    }
  }

  // 在局部距离内选择目标位置
  double len = (path[start_path_id] - current_pos).norm();
  for (int i = start_path_id + 1; i < (int)path.size(); i++) {
    len += (path[i] - path[i - 1]).norm();
    if (len > local_distance && (current_pos - path[i - 1]).norm() > 0.30) {
      target_pos = path[i - 1];
      break;
    }
  }

  return target_pos;
}

/**
 * @brief 计算最佳步骤
 * @param current_pos 当前位置
 * @param current_yaw 当前朝向
 * @param target_pos 目标位置
 * @return 最佳步骤向量
 *
 * 在多个候选步骤中选择代价最小的那个
 */
Vector2d ExplorationFSM::computeBestStep(
    const Vector2d& current_pos, double current_yaw, const Vector2d& target_pos)
{
  Vector2d best_step = target_pos;

  double min_cost = std::numeric_limits<double>::max();
  // 遍历所有候选动作步骤
  for (auto step : fp_->action_steps_) {
    double cost = computeActionTotalCost(current_pos, current_yaw, target_pos, step);
    if (cost < min_cost) {
      best_step = current_pos + step;
      min_cost = cost;
    }
  }

  return best_step;
}

/**
 * @brief 计算动作总代价
 * @param current_pos 当前位置
 * @param current_yaw 当前朝向
 * @param target_pos 目标位置
 * @param step 候选步骤
 * @return 总代价
 *
 * 计算执行某个步骤的总代价，包括：
 * - 距离目标的代价
 * - 距离变化的代价（奖励接近目标）
 * - 安全距离代价（惩罚靠近障碍物）
 */
double ExplorationFSM::computeActionTotalCost(const Vector2d& current_pos, double current_yaw,
    const Vector2d& target_pos, const Vector2d& step)
{
  // 各种代价权重
  const double traget_weight = FSMConstants::TARGET_WEIGHT;           // 目标距离权重
  const double traget_close_weight1 = FSMConstants::TARGET_CLOSE_WEIGHT_1;  // 接近目标权重1
  const double traget_close_weight2 = FSMConstants::TARGET_CLOSE_WEIGHT_2;  // 接近目标权重2
  const double safety_weight = FSMConstants::SAFETY_WEIGHT;           // 安全权重

  double cost = 0.0;

  // 1. 距离目标代价：距离越远代价越大
  Vector2d step_pos = current_pos + step;
  double target_cost = traget_weight * (step_pos - target_pos).norm();

  // 2. 距离变化代价：如果移动后距离目标增加则惩罚，减少则奖励
  double target_close_cost = (step_pos - target_pos).norm() - (current_pos - target_pos).norm();
  if (target_close_cost > 0)
    target_close_cost *= traget_close_weight1;  // 远离目标的惩罚
  else
    target_close_cost *= traget_close_weight2;  // 接近目标的奖励

  // 3. 安全距离代价：距离障碍物越近代价越大
  double safety_cost = safety_weight * computeActionSafetyCost(current_pos, step);

  cost += target_cost + target_close_cost + safety_cost;
  return cost;
}

/**
 * @brief 计算动作安全代价
 * @param current_pos 当前位置
 * @param step 执行的步骤
 * @return 安全代价
 *
 * 使用SDF距离计算沿步骤路径到障碍物的距离，越近代价越大
 */
double ExplorationFSM::computeActionSafetyCost(const Vector2d& current_pos, const Vector2d& step)
{
  const double min_safe_distance = FSMConstants::MIN_SAFE_DISTANCE;  // 最小安全距离
  const double sample_num = FSMConstants::SAMPLE_NUM;                // 采样点数量

  Vector2d dir = step;
  double len = dir.norm();
  dir.normalize();  // 单位化方向向量

  double safety_cost = 0.0;
  // 沿路径采样多个点计算安全代价
  for (double l = len / sample_num; l < len; l += len / sample_num) {
    Vector2d ckpt = current_pos + l * dir;  // 采样点位置
    Vector2d grad;
    double dist_to_occ = expl_manager_->sdf_map_->getDistWithGrad(ckpt, grad);  // 到障碍物的距离
    if (dist_to_occ < min_safe_distance)
      safety_cost += 1 / (dist_to_occ + 1e-2);  // 距离越近代价越大
  }

  return safety_cost;
}

/**
 * @brief 根据朝向差决定下一动作
 * @param current_yaw 当前朝向
 * @param target_yaw 目标朝向
 * @return 下一动作（左转、右转或前进）
 *
 * 使用动作角度阈值确定是否需要调整朝向
 */
int ExplorationFSM::decideNextAction(double current_yaw, double target_yaw)
{
  wrapAngle(target_yaw);  // 角度归一化到[-π, π]
  wrapAngle(current_yaw);
  double yaw_diff = target_yaw - current_yaw;
  wrapAngle(yaw_diff);    // 角度差归一化

  int next_action;
  // 如果角度差超过阈值，需要转弯
  if (std::fabs(yaw_diff) > FSMConstants::ACTION_ANGLE / 1.9) {
    if (yaw_diff > 0)
      next_action = ACTION::TURN_LEFT;   // 左转
    else
      next_action = ACTION::TURN_RIGHT;  // 右转
  }
  else
    next_action = ACTION::MOVE_FORWARD;  // 前进

  return next_action;
}

/**
 * @brief 可视化当前状态
 *
 * 绘制机器人位置、路径、目标点等信息到RViz
 */
void ExplorationFSM::visualize()
{
  auto ed_ptr = expl_manager_->ed_;

  // Lambda function to convert 2D vectors to 3D for visualization
  auto vec2dTo3d = [](const vector<Eigen::Vector2d>& vec2d, double z = 0.15) {
    vector<Eigen::Vector3d> vec3d;
    for (auto v : vec2d) vec3d.push_back(Vector3d(v(0), v(1), z));
    return vec3d;
  };

  // Draw frontier
  static int last_ftr2d_num = 0;
  for (int i = 0; i < (int)ed_ptr->frontiers_.size(); ++i) {
    visualization_->drawCubes(vec2dTo3d(ed_ptr->frontiers_[i]), fp_->vis_scale_,
        visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 1.0), "frontier", i, 4);
  }
  for (int i = ed_ptr->frontiers_.size(); i < last_ftr2d_num; ++i) {
    visualization_->drawCubes({}, fp_->vis_scale_, Vector4d(0, 0, 0, 1), "frontier", i, 4);
  }
  last_ftr2d_num = ed_ptr->frontiers_.size();

  // Draw dormant frontier
  static int last_dftr2d_num = 0;
  for (int i = 0; i < (int)ed_ptr->dormant_frontiers_.size(); ++i) {
    visualization_->drawCubes(vec2dTo3d(ed_ptr->dormant_frontiers_[i]), fp_->vis_scale_,
        Vector4d(0, 0, 0, 1), "dormant_frontier", i, 4);
  }
  for (int i = ed_ptr->dormant_frontiers_.size(); i < last_dftr2d_num; ++i) {
    visualization_->drawCubes({}, fp_->vis_scale_, Vector4d(0, 0, 0, 1), "dormant_frontier", i, 4);
  }
  last_dftr2d_num = ed_ptr->dormant_frontiers_.size();

  // Draw object
  // static int last_obj_num = 0;
  // for (int i = 0; i < (int)ed_ptr->objects_.size(); ++i) {
  //   visualization_->drawCubes(vec2dTo3d(ed_ptr->objects_[i]), fp_->vis_scale_,
  //       visualization_->getColor(double(i) / ed_ptr->objects_.size(), 1.0), "object", i, 4);
  // }
  // for (int i = ed_ptr->objects_.size(); i < last_obj_num; ++i) {
  //   visualization_->drawCubes({}, fp_->vis_scale_, Vector4d(0, 0, 0, 1), "object", i, 4);
  // }
  // last_obj_num = ed_ptr->objects_.size();

  static int last_obj_num = 0;
  for (int i = 0; i < (int)ed_ptr->objects_.size(); ++i) {
    int label = ed_ptr->object_labels_[i];
    visualization_->drawCubes(vec2dTo3d(ed_ptr->objects_[i]), fp_->vis_scale_,
        visualization_->getColor(double(label) / 5.0, 1.0), "object", i, 4);
  }
  for (int i = ed_ptr->objects_.size(); i < last_obj_num; ++i) {
    visualization_->drawCubes({}, fp_->vis_scale_, Vector4d(0, 0, 0, 1), "object", i, 4);
  }
  last_obj_num = ed_ptr->objects_.size();

  // Draw next best path
  visualization_->drawLines(vec2dTo3d(ed_ptr->next_best_path_), fp_->vis_scale_,
      Vector4d(1, 0.2, 0.2, 1), "next_path", 1, 6);

  // Draw next local point
  vector<Vector2d> local_points;
  local_points.push_back(fd_->local_pos_);
  visualization_->drawSpheres(vec2dTo3d(local_points), fp_->vis_scale_ * 3,
      Vector4d(0.2, 0.2, 1.0, 1), "local_point", 1, 6);

  visualization_->drawLines(vec2dTo3d(ed_ptr->tsp_tour_), fp_->vis_scale_ / 1.25,
      Vector4d(0.2, 1, 0.2, 1), "tsp_tour", 0, 6);

  visualization_->drawSpheres(vec2dTo3d(fd_->traveled_path_), fp_->vis_scale_ * 1.5,
      Vector4d(2.0 / 255.0, 111.0 / 255.0, 197.0 / 255.0, 1), "traveled_path", 1, 6);
}

void ExplorationFSM::clearVisMarker()
{
  auto ed_ptr = expl_manager_->ed_;
  for (int i = 0; i < 500; ++i) {
    visualization_->drawCubes({}, fp_->vis_scale_, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    visualization_->drawCubes({}, fp_->vis_scale_, Vector4d(0, 0, 0, 1), "dormant_frontier", i, 4);
    visualization_->drawCubes({}, fp_->vis_scale_, Vector4d(0, 0, 0, 1), "object", i, 4);
  }

  visualization_->drawLines({}, fp_->vis_scale_, Vector4d(0, 0, 1, 1), "next_path", 1, 6);
}

bool ExplorationFSM::updateFrontierAndObject()
{
  bool change_flag = false;
  auto frt_map = expl_manager_->frontier_map2d_;
  auto obj_map = expl_manager_->object_map2d_;
  auto ed = expl_manager_->ed_;
  Eigen::Vector2d start_pos2d = Eigen::Vector2d(fd_->start_pt_(0), fd_->start_pt_(1));

  change_flag = frt_map->isAnyFrontierChanged();
  frt_map->searchFrontiers();
  change_flag |= frt_map->dormantSeenFrontiers(start_pos2d, fd_->start_yaw_(0));
  frt_map->getFrontiers(ed->frontiers_, ed->frontier_averages_);
  frt_map->getDormantFrontiers(ed->dormant_frontiers_, ed->dormant_frontier_averages_);
  obj_map->getObjects(ed->objects_, ed->object_averages_, ed->object_labels_);

  return change_flag;
}

// Receive Habitat state messages
void ExplorationFSM::habitatStateCallback(const std_msgs::Int32ConstPtr& msg)
{
  if (msg->data == HABITAT_STATE::ACTION_FINISH && state_ == ROS_STATE::WAIT_ACTION_FINISH)
    transitState(PLAN_ACTION, "Habitat Finish Action");
  if (msg->data == HABITAT_STATE::EPISODE_FINISH)
    init(nh_);
  return;
}

// Periodically update frontiers and visualize in idle states
void ExplorationFSM::frontierCallback(const ros::TimerEvent& e)
{
  if (state_ != ROS_STATE::WAIT_TRIGGER && state_ != ROS_STATE::FINISH)
    return;

  updateFrontierAndObject();
  visualize();
}

// Receive user trigger to start exploration
void ExplorationFSM::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (state_ != ROS_STATE::WAIT_TRIGGER)
    return;
  fd_->trigger_ = true;
  cout << "Triggered!" << endl;
  transitState(PLAN_ACTION, "triggerCallback");
}

// Receive robot odometry and update traveled path + marker
void ExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg)
{
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  fd_->have_odom_ = true;

  Vector2d odom_pos2d = Vector2d(fd_->odom_pos_(0), fd_->odom_pos_(1));
  if (fd_->traveled_path_.empty())
    fd_->traveled_path_.push_back(odom_pos2d);
  else if ((fd_->traveled_path_.back() - odom_pos2d).norm() > 1e-2)
    fd_->traveled_path_.push_back(odom_pos2d);

  publishRobotMarker();
}

void ExplorationFSM::publishRobotMarker()
{
  const double robot_height = FSMConstants::ROBOT_HEIGHT;
  const double robot_radius = FSMConstants::ROBOT_RADIUS;

  // Create robot body cylinder marker
  visualization_msgs::Marker robot_marker;
  robot_marker.header.frame_id = "world";
  robot_marker.header.stamp = ros::Time::now();
  robot_marker.ns = "robot_position";
  robot_marker.id = 0;
  robot_marker.type = visualization_msgs::Marker::CYLINDER;
  robot_marker.action = visualization_msgs::Marker::ADD;

  // Set cylinder position
  robot_marker.pose.position.x = fd_->odom_pos_(0);
  robot_marker.pose.position.y = fd_->odom_pos_(1);
  robot_marker.pose.position.z = fd_->odom_pos_(2) + robot_height / 2.0;

  // Set cylinder orientation
  robot_marker.pose.orientation.x = fd_->odom_orient_.x();
  robot_marker.pose.orientation.y = fd_->odom_orient_.y();
  robot_marker.pose.orientation.z = fd_->odom_orient_.z();
  robot_marker.pose.orientation.w = fd_->odom_orient_.w();

  // Set cylinder dimensions
  robot_marker.scale.x = robot_radius * 2;  // Diameter
  robot_marker.scale.y = robot_radius * 2;  // Diameter
  robot_marker.scale.z = robot_height;      // Height

  // Set cylinder color (blue)
  robot_marker.color.r = 50.0 / 255.0;
  robot_marker.color.g = 50.0 / 255.0;
  robot_marker.color.b = 255.0 / 255.0;
  robot_marker.color.a = 1.0;

  // Create direction arrow marker
  visualization_msgs::Marker arrow_marker;
  arrow_marker.header.frame_id = "world";
  arrow_marker.header.stamp = ros::Time::now();
  arrow_marker.ns = "robot_direction";
  arrow_marker.id = 1;
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.action = visualization_msgs::Marker::ADD;

  // Set arrow position
  arrow_marker.pose.position.x = fd_->odom_pos_(0);
  arrow_marker.pose.position.y = fd_->odom_pos_(1);
  arrow_marker.pose.position.z = fd_->odom_pos_(2) + robot_height;

  // Set arrow orientation
  arrow_marker.pose.orientation.x = fd_->odom_orient_.x();
  arrow_marker.pose.orientation.y = fd_->odom_orient_.y();
  arrow_marker.pose.orientation.z = fd_->odom_orient_.z();
  arrow_marker.pose.orientation.w = fd_->odom_orient_.w();

  // Set arrow dimensions
  arrow_marker.scale.x = robot_radius + 0.13;  // Arrow length
  arrow_marker.scale.y = 0.08;                 // Arrow width
  arrow_marker.scale.z = 0.08;                 // Arrow thickness

  // Set arrow color (green)
  arrow_marker.color.r = 10.0 / 255.0;
  arrow_marker.color.g = 255.0 / 255.0;
  arrow_marker.color.b = 10.0 / 255.0;
  arrow_marker.color.a = 1.0;

  // Publish both markers
  robot_marker_pub_.publish(robot_marker);
  robot_marker_pub_.publish(arrow_marker);
}

void ExplorationFSM::confidenceThresholdCallback(const std_msgs::Float64ConstPtr& msg)
{
  if (fd_->have_confidence_)
    return;
  fd_->have_confidence_ = true;
  expl_manager_->sdf_map_->object_map2d_->setConfidenceThreshold(msg->data);
}

// Transition FSM state and log the change
void ExplorationFSM::transitState(ROS_STATE new_state, string pos_call)
{
  int pre_s = int(state_);
  state_ = new_state;
  cout << "[ " + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " +
              fd_->state_str_[int(new_state)]
       << endl;
}
}  // namespace apexnav_planner
