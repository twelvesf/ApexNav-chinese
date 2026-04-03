#include <exploration_manager/exploration_manager.h>
#include <exploration_manager/exploration_fsm_traj.h>
#include <exploration_manager/exploration_data.h>
#include <vis_utils/planning_visualization.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

namespace apexnav_planner {

void ExplorationFSMReal::init(ros::NodeHandle& nh)
{
  nh_ = nh;
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /* Initialize main modules */
  expl_manager_.reset(new ExplorationManager);
  expl_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));
  fp_->vis_scale_ = expl_manager_->sdf_map_->getResolution() * FSMConstantsReal::VIS_SCALE_FACTOR;

  state_ = RealFSM::State::INIT;

  // Load real-world specific parameters
  nh.param("fsm/replan_time", fp_->replan_time_, 0.2);
  nh.param("fsm/replan_traj_end_threshold", fp_->replan_traj_end_threshold_, 1.0);
  nh.param("fsm/replan_frontier_change_delay", fp_->replan_frontier_change_delay_, 0.5);
  nh.param("fsm/replan_timeout", fp_->replan_timeout_, 2.0);

  /* ROS Timer 在.h文件中设置*/
  exec_timer_ = nh.createTimer(
      ros::Duration(FSMConstantsReal::EXEC_TIMER_DURATION), &ExplorationFSMReal::FSMCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(FSMConstantsReal::FRONTIER_TIMER_DURATION),
      &ExplorationFSMReal::frontierCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &ExplorationFSMReal::safetyCallback, this);

  /* ROS Subscriber */
  // 触发
  trigger_sub_ =
      nh.subscribe("/move_base_simple/goal", 10, &ExplorationFSMReal::triggerCallback, this);
  // 基本用不上，用于手动给一个点，然后非过去
  goal_sub_ = nh.subscribe("/initialpose", 10, &ExplorationFSMReal::goalCallback, this);
  // 订阅里程计
  odom_sub_ = nh.subscribe("/odom_world", 10, &ExplorationFSMReal::odometryCallback, this);
  // 订阅检测置信度阈值
  confidence_threshold_sub_ = nh.subscribe(
      "/detector/confidence_threshold", 10, &ExplorationFSMReal::confidenceThresholdCallback, this);

  /* ROS Publisher */
  // /ros/state：FSM 当前阶段
  // /ros/expl_state：这版里没真正用
  // /ros/expl_result：当前规划结果类别
  // /robot：机器人可视化 marker
  ros_state_pub_ = nh.advertise<std_msgs::Int32>("/ros/state", 10);
  expl_state_pub_ = nh.advertise<std_msgs::Int32>("/ros/expl_state", 10);
  expl_result_pub_ = nh.advertise<std_msgs::Int32>("/ros/expl_result", 10);
  robot_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/robot", 10);

  // Real-world trajectory publishers
  // 发布轨迹信息
  // 发布stop命令
  poly_traj_pub_ = nh.advertise<trajectory_manager::PolyTraj>("/planning/trajectory", 10);
  stop_pub_ = nh.advertise<std_msgs::Empty>("/traj_server/stop", 10);

  ROS_INFO("[ExplorationFSMReal] Initialization complete.");
}

// Main FSM callback for real-world exploration
void ExplorationFSMReal::FSMCallback(const ros::TimerEvent& e)
{
  exec_timer_.stop();

  // Publish current state
  std_msgs::Int32 ros_state_msg;
  ros_state_msg.data = static_cast<int>(state_);
  ros_state_pub_.publish(ros_state_msg);

  switch (state_) {
    case RealFSM::State::INIT: {
      // Wait for odometry and target confidence threshold
      if (!fd_->have_odom_ || !fd_->have_confidence_) {
        ROS_WARN_THROTTLE(1.0, "[Real] No odom || No target confidence threshold.");
        exec_timer_.start();
        return;
      }
      // Go to WAIT_TRIGGER when prerequisites are ready
      clearVisMarker();
      transitState(RealFSM::State::WAIT_TRIGGER, "FSM");
      break;
    }

    case RealFSM::State::WAIT_TRIGGER: {
      // Do nothing but wait for trigger
      ROS_WARN_THROTTLE(1.0, "[Real] Waiting for trigger...");
      break;
    }

    case RealFSM::State::FINISH: {
      fd_->static_state_ = true;
      if (!fd_->have_finished_) {
        fd_->have_finished_ = true;
        clearVisMarker();
      }
      ROS_WARN_THROTTLE(1.0, "[Real] Finish exploration!");
      break;
    }

    case RealFSM::State::PLAN_TRAJ: {
      // Plan trajectory based on current state
      if (fd_->static_state_) {
        // Robot is static, use current odometry
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_yaw_(0) = fd_->odom_yaw_;
        fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
      }
      else {
        // Robot is moving, predict future state for smooth replanning
        LocalTrajectory* info = &expl_manager_->gcopter_->local_trajectory_;
        double t_plan = (ros::Time::now() - info->start_time).toSec() + fp_->replan_time_;
        t_plan = min(t_plan, info->duration);

        Eigen::Vector3d cur_pos = info->traj.getPos(t_plan);
        Eigen::Vector3d cur_vel = info->traj.getVel(t_plan);
        Eigen::Vector3d cur_acc = info->traj.getAcc(t_plan);
        //通过速度反推当前航向角
        //yaw=atan2(vy,vx)
        double cur_yaw = atan2(cur_vel(1), cur_vel(0));

        // Calculate yaw rate from acceleration
        Eigen::Matrix2d B_h;
        B_h << 0, -1.0, 1.0, 0;
        //B_h矩阵作用是把一个2d向量旋转90度，[vx, vy] 变成 [-vy, vx]
        //cur_vel_2d = [vx, vy]
        // cur_acc_2d = [ax, ay]
        Eigen::Vector2d cur_vel_2d = cur_vel.head(2);
        Eigen::Vector2d cur_acc_2d = cur_acc.head(2);
        //norm_vel=|v|= sqrt(vx^2 + vy^2)
        double norm_vel = cur_vel_2d.norm();
        //1 / (vx^2 + vy^2 + 1e-2) ，1e-2 是为了防止速度太小时分母接近 0，避免数值爆炸
        double help1 = 1.0 / (norm_vel * norm_vel + 1e-2);
        //omega = (vx * ay - vy * ax) / (vx^2 + vy^2 + 1e-2)    其实就是cur_yaw=atan2(vy,vx)的导数
        double omega = help1 * cur_acc_2d.transpose() * B_h * cur_vel_2d;

        fd_->start_pt_ = cur_pos;
        fd_->start_vel_ = cur_vel;
        fd_->start_yaw_(0) = cur_yaw;
        fd_->start_yaw_(1) = omega;
      }

      //根据当前机器人状态和当前探索结果，选出下一个局部目标，并真正生成一条可执行轨迹。
      TrajPlannerResult res = callTrajectoryPlanner();

      if (res == TrajPlannerResult::FAILED) {
        ROS_WARN("[Real] Plan trajectory failed");
        fd_->static_state_ = true;
      }
      else if (res == TrajPlannerResult::SUCCESS) {
        transitState(RealFSM::State::EXEC_TRAJ, "FSM");
      }
      else {  // TrajPlannerResult::MISSION_COMPLETE
        transitState(RealFSM::State::FINISH, "FSM");
      }
      //可视化
      visualize();
      break;
    }
    //发布规划好的轨迹
    case RealFSM::State::EXEC_TRAJ: {
      // Publish trajectory and transition to execution monitoring
      double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
      if (dt > 0) {
        trajectory_manager::PolyTraj poly_msg;
        polyTraj2ROSMsg(fd_->newest_traj_, poly_msg);
        poly_traj_pub_.publish(poly_msg);
        fd_->static_state_ = false;
        transitState(RealFSM::State::REPLAN, "FSM");
      }
      break;
    }

    case RealFSM::State::REPLAN: {
      // Monitor trajectory execution and decide when to replan
      LocalTrajectory* info = &expl_manager_->gcopter_->local_trajectory_;
      double t_cur = (ros::Time::now() - info->start_time).toSec();
      double time_to_end = info->duration - t_cur;

      // Replan if trajectory is almost finished
      //当前轨迹还剩多久结束时就提前触发下一次重规划
      if (time_to_end < fp_->replan_traj_end_threshold_) {
        transitState(RealFSM::State::PLAN_TRAJ, "FSM");
        ROS_WARN("[Real] Replan: traj fully executed");
        exec_timer_.start();
        return;
      }

      // Replan if frontier changed during exploration
      if (t_cur > fp_->replan_frontier_change_delay_ &&
          fd_->final_result_ == FINAL_RESULT::EXPLORE &&    //处于几何探索阶段 frotiner改变
          expl_manager_->frontier_map2d_->isAnyFrontierChanged()) {
        transitState(RealFSM::State::PLAN_TRAJ, "FSM");
        ROS_WARN("[Real] Replan: frontier changed");
        exec_timer_.start();
        return;
      }

      // Replan if trajectory timeout //超时
      if (t_cur > fp_->replan_timeout_) {
        transitState(RealFSM::State::PLAN_TRAJ, "FSM");
        ROS_WARN("[Real] Replan: time out");
        exec_timer_.start();
        return;
      }
      break;
    }
  }

  exec_timer_.start();
}

//根据当前机器人状态和当前探索结果，选出下一个局部目标，并真正生成一条可执行轨迹。
TrajPlannerResult ExplorationFSMReal::callTrajectoryPlanner()
{
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);
  //更新frontier
  updateFrontierAndObject();

  // Call exploration manager to find next best point //决定下一步应该去追哪个目标，以及给出一条 2D 粗路径。
  //传的是规划是否成功的状态
  int expl_res = expl_manager_->planNextBestPoint(fd_->start_pt_, fd_->start_yaw_(0));

  // Determine final result based on exploration result
  //几何探索
  if (expl_res == EXPL_RESULT::EXPLORATION)
    fd_->final_result_ = FINAL_RESULT::EXPLORE;
  else if (expl_res == EXPL_RESULT::NO_COVERABLE_FRONTIER ||
           expl_res == EXPL_RESULT::NO_PASSABLE_FRONTIER)
    fd_->final_result_ = FINAL_RESULT::NO_FRONTIER;
  else
  //语义搜索
    fd_->final_result_ = FINAL_RESULT::SEARCH_OBJECT;

  // Publish exploration result
  std_msgs::Int32 expl_result_msg;
  expl_result_msg.data = fd_->final_result_;
  expl_result_pub_.publish(expl_result_msg);

  if (fd_->final_result_ == FINAL_RESULT::NO_FRONTIER) {
    ROS_WARN("[Real] No (passable) frontier");
    return TrajPlannerResult::MISSION_COMPLETE;
  }

  // Select local target from global path
  Eigen::Vector2d goal_pos = expl_manager_->ed_->next_pos_;
  double goal_yaw = 0.0;
  auto path = expl_manager_->ed_->next_best_path_;
  // 根据当前位置和全局路径，选取局部目标点及朝向
  selectLocalTarget(fd_->start_pt_.head(2), path, 4.0, goal_pos, goal_yaw);

  // Check if reached object
  if (fd_->final_result_ == FINAL_RESULT::SEARCH_OBJECT &&
      (fd_->start_pt_.head(2) - goal_pos).norm() < 0.25) {
    ROS_ERROR("[Real] Reach the object successfully!");
    return TrajPlannerResult::MISSION_COMPLETE;
  }

  // Prepare state for trajectory planning
  Eigen::VectorXd goal_state(5), current_state(5);
  Eigen::Vector3d current_control(0.0, 0.0, 0.0);
  //速度标量大小
  double start_vel = Eigen::Vector2d(fd_->start_vel_(0), fd_->start_vel_(1)).norm();
  current_state << fd_->start_pt_(0), fd_->start_pt_(1), fd_->start_yaw_(0), 0.0, start_vel;
  goal_state << goal_pos(0), goal_pos(1), goal_yaw, 0.0, 0.0;

  // Plan trajectory using GCopter
  //开始真正做轨迹规划。
  bool traj_res = expl_manager_->planTrajectory(current_state, goal_state, current_control);
  if (traj_res) {
    //拿到的是 GCopter 里保存的那条最新局部轨迹
    auto info = &expl_manager_->gcopter_->local_trajectory_;
    //给这条轨迹确定一个实际生效的起始时间。
    info->start_time = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;
    //把本轮规划结果正式交给 FSM 保存。
    //fd_ 里主要存 FSM 当前执行状态以及最新的局部连续轨迹
    fd_->newest_traj_ = expl_manager_->gcopter_->local_trajectory_;
    return TrajPlannerResult::SUCCESS;
  }

  return TrajPlannerResult::FAILED;
}

void ExplorationFSMReal::polyTraj2ROSMsg(
    const LocalTrajectory& local_traj, trajectory_manager::PolyTraj& poly_msg)
{
  auto data = &local_traj;
  Eigen::VectorXd durs = data->traj.getDurations();
  int piece_num = data->traj.getPieceNum();

  poly_msg.drone_id = 0;
  poly_msg.traj_id = data->traj_id;
  poly_msg.start_time = data->start_time;
  poly_msg.order = 7;
  poly_msg.duration.resize(piece_num);
  poly_msg.coef_x.resize(8 * piece_num);
  poly_msg.coef_y.resize(8 * piece_num);
  poly_msg.coef_z.resize(8 * piece_num);

  for (int i = 0; i < piece_num; ++i) {
    poly_msg.duration[i] = durs(i);

    auto cMat = data->traj.operator[](i).getCoeffMat();
    int i8 = i * 8;
    for (int j = 0; j < 8; j++) {
      poly_msg.coef_x[i8 + j] = cMat(0, j);
      poly_msg.coef_y[i8 + j] = cMat(1, j);
      poly_msg.coef_z[i8 + j] = cMat(2, j);
    }
  }
}
//把全局粗路径转换成一个当前可执行、朝向合理、并且更安全的局部跟踪目标。
//结果存到：expl_manager_->ed_->next_local_pos_
//用 ESDF 距离场做一个局部避障修正：如果局部目标点太贴近障碍，就沿“离障碍更远”的方向把它推开一点。
void ExplorationFSMReal::selectLocalTarget(const Eigen::Vector2d& current_pos,
    const std::vector<Eigen::Vector2d>& path, const double& local_distance,
    Eigen::Vector2d& target_pos, double& target_yaw)
{
  // First, try to find a collision-free target from the end of path
  //从路径尾部往前扫，找一个在给定朝向下不会碰撞的位置，作为初始target_pos
  //这里的朝向是从该点看向路径终点的方向
  for (int i = path.size() - 2; i >= 0; i--) {
    target_yaw = atan2(path.back()(1) - path[i](1), path.back()(0) - path[i](0));
    if (!expl_manager_->kinoastar_->isCollisionPosYaw(path[i], target_yaw)) {
      target_pos = path[i];
      break;
    }
  }

  // Find closest path point to current position
  //找到机器人在全局路径的位置
  int start_path_id = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < (int)path.size() - 1; i++) {
    Eigen::Vector2d pos = path[i];
    if ((pos - current_pos).norm() < min_dist) {
      min_dist = (pos - current_pos).norm();
      start_path_id = i + 1;
    }
  }

  // Select local target within local_distance
  //按照local_distance选一个局部前视目标点，全局路径 -> 当前只跟踪前方一小段
  double len = (path[start_path_id] - current_pos).norm();
  for (int i = start_path_id + 1; i < (int)path.size(); i++) {
    len += (path[i] - path[i - 1]).norm();
    if (len > local_distance && (current_pos - path[i - 1]).norm() > 0.30) {
      target_pos = path[i - 1];
      target_yaw = atan2(path[i](1) - path[i - 1](1), path[i](0) - path[i - 1](0));
      break;
    }
  }

  // Gradient-based safety adjustment
//   每次最多挪 0.05m
// 挪动太小就认为收敛
// 最多迭代 30 次
  double step_size = 0.05;
  double tolerance = 1e-3;
  int max_iterations = 30;

  for (int i = 0; i < max_iterations; ++i) {
    Eigen::Vector2d prev_pos = target_pos;

    // Get gradient from SDF map
    Eigen::Vector2d grad;
    //查询当前点的障碍距离和梯度，disk：当前target_pos离最近障碍的距离，grad：离障碍更远的方向
    double dist = expl_manager_->sdf_map_->getDistWithGrad(target_pos, grad);

    if (dist > 0.26)
      break;

    // Move along gradient to safer position
    //朝更空旷、更远离障碍的方向小步移动。
    if (grad.norm() > 1e-6) {
      //grad.normalized:把向量 grad 归一化成单位向量。方向不变，长度为1
      target_pos += step_size * grad.normalized();
    }

    // Check convergence
    //位置变化很小，认为收敛
    if ((target_pos - prev_pos).norm() < tolerance) {
      break;
    }
  }

  // Store selected local target
  expl_manager_->ed_->next_local_pos_ = target_pos;
}
//当前 frontier、dormant frontier、object、下一条粗路径、
//局部目标点和 TSP 巡回路线全部发布成 RViz 可视化标记
void ExplorationFSMReal::visualize()
{
  auto ed_ptr = expl_manager_->ed_;

  auto vec2dTo3d = [](const std::vector<Eigen::Vector2d>& vec2d, double z = 0.15) {
    std::vector<Eigen::Vector3d> vec3d;
    for (auto v : vec2d) vec3d.push_back(Eigen::Vector3d(v(0), v(1), z));
    return vec3d;
  };

  // Draw frontiers
  static int last_ftr2d_num = 0;
  for (int i = 0; i < (int)ed_ptr->frontiers_.size(); ++i) {
    visualization_->drawCubes(vec2dTo3d(ed_ptr->frontiers_[i]), fp_->vis_scale_,
        visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 1.0), "frontier", i, 4);
  }
  for (int i = ed_ptr->frontiers_.size(); i < last_ftr2d_num; ++i) {
    visualization_->drawCubes({}, fp_->vis_scale_, Eigen::Vector4d(0, 0, 0, 1), "frontier", i, 4);
  }
  last_ftr2d_num = ed_ptr->frontiers_.size();

  // Draw dormant frontiers
  static int last_dftr2d_num = 0;
  for (int i = 0; i < (int)ed_ptr->dormant_frontiers_.size(); ++i) {
    visualization_->drawCubes(vec2dTo3d(ed_ptr->dormant_frontiers_[i]), fp_->vis_scale_,
        Eigen::Vector4d(0, 0, 0, 1), "dormant_frontier", i, 4);
  }
  for (int i = ed_ptr->dormant_frontiers_.size(); i < last_dftr2d_num; ++i) {
    visualization_->drawCubes(
        {}, fp_->vis_scale_, Eigen::Vector4d(0, 0, 0, 1), "dormant_frontier", i, 4);
  }
  last_dftr2d_num = ed_ptr->dormant_frontiers_.size();

  // Draw objects
  static int last_obj_num = 0;
  for (int i = 0; i < (int)ed_ptr->objects_.size(); ++i) {
    int label = ed_ptr->object_labels_[i];
    visualization_->drawCubes(vec2dTo3d(ed_ptr->objects_[i]), fp_->vis_scale_,
        visualization_->getColor(double(label) / 5.0, 1.0), "object", i, 4);
  }
  for (int i = ed_ptr->objects_.size(); i < last_obj_num; ++i) {
    visualization_->drawCubes({}, fp_->vis_scale_, Eigen::Vector4d(0, 0, 0, 1), "object", i, 4);
  }
  last_obj_num = ed_ptr->objects_.size();

  // Draw next best path
  visualization_->drawLines(vec2dTo3d(ed_ptr->next_best_path_), fp_->vis_scale_,
      Eigen::Vector4d(1, 0.2, 0.2, 1), "next_path", 1, 6);

  // Draw next local point
  std::vector<Eigen::Vector2d> local_points;
  local_points.push_back(ed_ptr->next_local_pos_);
  visualization_->drawSpheres(vec2dTo3d(local_points), fp_->vis_scale_ * 3,
      Eigen::Vector4d(0.2, 0.2, 1.0, 1), "local_point", 1, 6);

  visualization_->drawLines(vec2dTo3d(ed_ptr->tsp_tour_), fp_->vis_scale_ / 1.25,
      Eigen::Vector4d(0.2, 1, 0.2, 1), "tsp_tour", 0, 6);
}
//发布空 marker，把 RViz 里之前画出来的 frontier / object / path 可视化全部清掉
void ExplorationFSMReal::clearVisMarker()
{
  for (int i = 0; i < 500; ++i) {
    visualization_->drawCubes({}, fp_->vis_scale_, Eigen::Vector4d(0, 0, 0, 1), "frontier", i, 4);
    visualization_->drawCubes(
        {}, fp_->vis_scale_, Eigen::Vector4d(0, 0, 0, 1), "dormant_frontier", i, 4);
    visualization_->drawCubes({}, fp_->vis_scale_, Eigen::Vector4d(0, 0, 0, 1), "object", i, 4);
  }
  visualization_->drawLines({}, fp_->vis_scale_, Eigen::Vector4d(0, 0, 1, 1), "next_path", 1, 6);
}
// 状态同步/刷新函数：

// 把当前地图里的 frontier 和 object 信息，重新整理到 ExplorationData ed_ 里，供后面的规划和可视化使用。
bool ExplorationFSMReal::updateFrontierAndObject()
{
  bool change_flag = false;
  auto frt_map = expl_manager_->frontier_map2d_;
  auto obj_map = expl_manager_->object_map2d_;
  //智能指针
  auto ed = expl_manager_->ed_;
  Eigen::Vector2d sensor_pos = Eigen::Vector2d(fd_->odom_pos_(0), fd_->odom_pos_(1));
// 检查“已有 frontier 是否因为地图更新而失效/变化”
  change_flag = frt_map->isAnyFrontierChanged();
  // 重建 active frontier 列表。
  frt_map->searchFrontiers();
  // 把不值得继续作为主探索目标的 frontier 暂时降级。
  change_flag |= frt_map->dormantSeenFrontiers(sensor_pos, fd_->odom_yaw_);
  //拷贝到ed
  frt_map->getFrontiers(ed->frontiers_, ed->frontier_averages_);
  frt_map->getDormantFrontiers(ed->dormant_frontiers_, ed->dormant_frontier_averages_);
  obj_map->getObjects(ed->objects_, ed->object_averages_, ed->object_labels_);

  return change_flag;
}

void ExplorationFSMReal::frontierCallback(const ros::TimerEvent& e)
{
  // Update frontiers and visualize in idle states
  if (state_ != RealFSM::State::WAIT_TRIGGER && state_ != RealFSM::State::FINISH)
    return;

  updateFrontierAndObject();
  visualize();
}

void ExplorationFSMReal::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (state_ != RealFSM::State::WAIT_TRIGGER)
    return;

  fd_->trigger_ = true;
  ROS_INFO("[Real] Exploration triggered!");
  transitState(RealFSM::State::PLAN_TRAJ, "triggerCallback");
}

void ExplorationFSMReal::odometryCallback(const nav_msgs::OdometryConstPtr& msg)
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

  // Extract linear velocity
  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  // Extract angular velocity
  fd_->odom_omega_(0) = msg->twist.twist.angular.x;
  fd_->odom_omega_(1) = msg->twist.twist.angular.y;
  fd_->odom_omega_(2) = msg->twist.twist.angular.z;

  fd_->have_odom_ = true;

  // Publish robot marker for visualization
  //在 RViz 里把机器人当前位置和朝向画出来。
  publishRobotMarker();
}
//从外部话题 /detector/confidence_threshold 接收一次“目标置信度阈值”，并把它写进 ObjectMap2D
void ExplorationFSMReal::confidenceThresholdCallback(const std_msgs::Float64ConstPtr& msg)
{
  if (fd_->have_confidence_)
    return;
  fd_->have_confidence_ = true;
  expl_manager_->sdf_map_->object_map2d_->setConfidenceThreshold(msg->data);
  ROS_INFO("[Real] Confidence threshold set to: %.2f", msg->data);
}

//无关，给一个点导航
void ExplorationFSMReal::goalCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  Eigen::VectorXd goal_state(5), current_state(5);
  Eigen::Vector3d current_control;
  current_state << fd_->odom_pos_(0), fd_->odom_pos_(1), fd_->odom_yaw_, 0.0, fd_->odom_vel_(0);
  goal_state << x, y, yaw, 0.0, 0.0;
  if ((current_state.head(2) - goal_state.head(2)).norm() > 0.2) {
    current_control << 0.0, 0.0, 0.0;
    expl_manager_->planTrajectory(current_state, goal_state, current_control);
    trajectory_manager::PolyTraj poly_msg;
    polyTraj2ROSMsg(expl_manager_->gcopter_->local_trajectory_, poly_msg);
    poly_traj_pub_.publish(poly_msg);
  }
  ROS_INFO("[Real] Received goal pose: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
}

//紧急停车
//发布stop信息给traj_server
void ExplorationFSMReal::emergencyStop()
{
  fd_->static_state_ = true;
  stop_pub_.publish(std_msgs::Empty());
}
//在机器人执行轨迹期间，持续做安全监控；一旦发现偏离轨迹太多或者前方轨迹会撞障碍，就紧急停车并重新规划。
void ExplorationFSMReal::safetyCallback(const ros::TimerEvent& e)
{
  if (state_ != RealFSM::State::REPLAN)
    return;

  // Check if robot deviates from planned trajectory
  double t_cur = (ros::Time::now() - expl_manager_->gcopter_->local_trajectory_.start_time).toSec();
  t_cur = min(t_cur, expl_manager_->gcopter_->local_trajectory_.duration);
  Eigen::Vector3d cur_pos = expl_manager_->gcopter_->local_trajectory_.traj.getPos(t_cur);

  if ((cur_pos.head(2) - fd_->odom_pos_.head(2)).norm() > 0.3) {
    ROS_ERROR("[Real] Odom far from traj (%.2f, %.2f), Stop!!!", cur_pos(0), cur_pos(1));
    emergencyStop();
    transitState(RealFSM::State::PLAN_TRAJ, "Odom Far From Trajectory");
    return;
  }

  // Time-sampled safety check - use inflated map to detect obstacles
  double time_horizon = 2.5;  // Check trajectory for next 2.5 seconds
  double sample_dt = 0.1;     // Sample every 0.1 seconds
//检查未来一小段轨迹上是否会碰到障碍
//这里会对未来 2.5s 的轨迹做采样检查，每 0.1s 看一个点。
  for (double t_check = t_cur;
      t_check <= min(t_cur + time_horizon, expl_manager_->gcopter_->local_trajectory_.duration);
      t_check += sample_dt) {
    Eigen::Vector3d check_pos = expl_manager_->gcopter_->local_trajectory_.traj.getPos(t_check);
    Eigen::Vector2d check_pos_2d = check_pos.head(2);

    // Skip positions too close to origin
    if ((check_pos_2d - Eigen::Vector2d(0.0, 0.0)).norm() < 1.5)
      continue;

    if (expl_manager_->sdf_map_->getInflateOccupancy(check_pos_2d)) {
      ROS_ERROR("[Real] Safety Stop!!! Obstacle detected (%.2f, %.2f) at time %.2f",
          check_pos_2d(0), check_pos_2d(1), t_check);
      emergencyStop();
      transitState(RealFSM::State::PLAN_TRAJ, "Trajectory Safety Stop");
      break;
    }
  }
}
//在 RViz 里把机器人当前位置和朝向画出来。
void ExplorationFSMReal::publishRobotMarker()
{
  const double robot_height = FSMConstantsReal::ROBOT_HEIGHT;
  const double robot_radius = FSMConstantsReal::ROBOT_RADIUS;

  // Create robot body cylinder marker
  visualization_msgs::Marker robot_marker;
  robot_marker.header.frame_id = "world";
  robot_marker.header.stamp = ros::Time::now();
  robot_marker.ns = "robot_position";
  robot_marker.id = 0;
  robot_marker.type = visualization_msgs::Marker::CYLINDER;
  robot_marker.action = visualization_msgs::Marker::ADD;

  robot_marker.pose.position.x = fd_->odom_pos_(0);
  robot_marker.pose.position.y = fd_->odom_pos_(1);
  robot_marker.pose.position.z = fd_->odom_pos_(2) + robot_height / 2.0;

  robot_marker.pose.orientation.x = fd_->odom_orient_.x();
  robot_marker.pose.orientation.y = fd_->odom_orient_.y();
  robot_marker.pose.orientation.z = fd_->odom_orient_.z();
  robot_marker.pose.orientation.w = fd_->odom_orient_.w();

  robot_marker.scale.x = robot_radius * 2;
  robot_marker.scale.y = robot_radius * 2;
  robot_marker.scale.z = robot_height;

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

  arrow_marker.pose.position.x = fd_->odom_pos_(0);
  arrow_marker.pose.position.y = fd_->odom_pos_(1);
  arrow_marker.pose.position.z = fd_->odom_pos_(2) + robot_height;

  arrow_marker.pose.orientation.x = fd_->odom_orient_.x();
  arrow_marker.pose.orientation.y = fd_->odom_orient_.y();
  arrow_marker.pose.orientation.z = fd_->odom_orient_.z();
  arrow_marker.pose.orientation.w = fd_->odom_orient_.w();

  arrow_marker.scale.x = robot_radius + 0.13;
  arrow_marker.scale.y = 0.08;
  arrow_marker.scale.z = 0.08;

  arrow_marker.color.r = 10.0 / 255.0;
  arrow_marker.color.g = 255.0 / 255.0;
  arrow_marker.color.b = 10.0 / 255.0;
  arrow_marker.color.a = 1.0;

  robot_marker_pub_.publish(robot_marker);
  robot_marker_pub_.publish(arrow_marker);
}

void ExplorationFSMReal::transitState(RealFSM::State new_state, std::string pos_call)
{
  std::string state_str[] = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "EXEC_TRAJ", "REPLAN",
    "FINISH" };
  ROS_INFO("[Real FSM]: %s -> from %s to %s", pos_call.c_str(),
      state_str[static_cast<int>(state_)].c_str(), state_str[static_cast<int>(new_state)].c_str());
  state_ = new_state;
}

}  // namespace apexnav_planner
