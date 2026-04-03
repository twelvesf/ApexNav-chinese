/**
 * @file exploration_node.cpp
 * @brief 主入口节点，根据运行模式启动相应的探索有限状态机
 * @author ApexNav Team
 *
 * 该节点是ApexNav探索系统的入口点，负责根据配置参数选择
 * 仿真模式或真实世界模式的有限状态机进行自主探索任务。
 */

#include <ros/ros.h>
#include <exploration_manager/exploration_fsm.h>
#include <exploration_manager/exploration_fsm_traj.h>

#include <exploration_manager/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace apexnav_planner;

/**
 * @brief 主函数：初始化ROS节点并根据模式启动相应的探索FSM
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序退出码
 */
int main(int argc, char** argv)
{
  // 初始化ROS节点，节点名为"apexnav_node"
  ros::init(argc, argv, "apexnav_node");
  ros::NodeHandle nh("~");

  // 检查是否为真实世界模式（默认为仿真模式）
  bool is_real_world = false;
  nh.param("is_real_world", is_real_world, false);

  if (is_real_world) {
    // 真实世界模式：使用连续轨迹规划的FSM
    ROS_INFO("========================================");
    ROS_INFO("  Starting in REAL WORLD mode");
    ROS_INFO("========================================");
    ExplorationFSMReal expl_fsm;
    expl_fsm.init(nh);
    ros::Duration(1.0).sleep();
    ros::spin();
  }
  else {
    // 仿真模式：使用离散动作控制的FSM
    ROS_INFO("========================================");
    ROS_INFO("  Starting in SIMULATION mode");
    ROS_INFO("========================================");
    ExplorationFSM expl_fsm;
    expl_fsm.init(nh);
    ros::Duration(1.0).sleep();
    ros::spin();
  }

  return 0;
}
