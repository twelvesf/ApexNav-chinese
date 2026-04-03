"""
Habitat Velocity Control with ROS Integration

这个脚本实现了一个机器人导航模拟系统，使用Habitat模拟器创建虚拟环境，
并通过ROS (Robot Operating System) 进行实时控制和数据发布。

主要功能：
- 在虚拟环境中进行对象导航任务
- 接收外部速度控制命令 (/cmd_vel)
- 发布传感器观察数据和状态信息
- 支持实时控制和可视化

作者: [未指定]
日期: 2026年3月10日
"""

import os
import signal
import gzip
import json
import time

import habitat
import numpy as np
from habitat.sims.habitat_simulator.actions import HabitatSimActions
from omegaconf import DictConfig
from habitat.config.default import patch_config
import hydra  # noqa
from habitat2ros import habitat_publisher
import rospy
from copy import deepcopy
from std_msgs.msg import Float64, String
from vlm.Labels import MP3D_ID_TO_NAME
from geometry_msgs.msg import Twist
import habitat_sim
from habitat_sim.utils import common as utils

from habitat.config.default_structured_configs import (
    CollisionsMeasurementConfig,
    FogOfWarConfig,
    TopDownMapMeasurementConfig,
)
from habitat.utils.visualizations.utils import observations_to_image


def signal_handler(sig, frame):
    """
    信号处理器：处理Ctrl+C中断信号

    当用户按Ctrl+C时，优雅地关闭ROS节点并退出程序

    参数:
        sig: 信号编号
        frame: 当前栈帧
    """
    print("Ctrl+C detected! Shutting down...")
    rospy.signal_shutdown("Manual shutdown")
    os._exit(0)


def transform_rgb_bgr(image):
    """
    将RGB图像转换为BGR格式

    Habitat默认使用RGB，但某些处理可能需要BGR格式

    参数:
        image: RGB格式的图像数组

    返回:
        BGR格式的图像数组
    """
    return image[:, :, [2, 1, 0]]


def publish_observations(event):
    """
    定时发布观察数据的回调函数

    通过ROS发布当前的观察数据和融合分数

    参数:
        event: 定时器事件
    """
    global msg_observations, fusion_score
    global ros_pub, confidence_threshold_pub
    tmp = deepcopy(msg_observations)
    ros_pub.habitat_publish_ros_topic(tmp)
    msg = Float64()
    msg.data = fusion_score
    confidence_threshold_pub.publish(msg)


def cmd_vel_callback(msg):
    """
    速度控制命令的回调函数

    从/cmd_vel话题接收线速度和角速度命令

    参数:
        msg: Twist消息，包含线速度和角速度
    """
    global cmd_vel, cmd_omega
    cmd_vel = msg.linear.x
    cmd_omega = msg.angular.z


@hydra.main(
    version_base=None,
    config_path="config",
    config_name="habitat_vel_control",
)
def main(cfg: DictConfig) -> None:
    """
    主函数：初始化Habitat环境并运行导航控制循环

    使用Hydra进行配置管理，创建Habitat环境，设置ROS通信，
    并在主循环中处理速度控制和观察发布

    参数:
        cfg: Hydra配置对象，包含所有实验参数
    """
    global msg_observations, fusion_score
    global ros_pub, confidence_threshold_pub
    global obj_point_cloud
    global obj_point_cloud_pub
    global cmd_vel, cmd_omega

    # 初始化速度控制变量
    cmd_vel = 0.0
    cmd_omega = 0.0

    # 加载验证数据集，获取类别映射
    with gzip.open(
        "data/datasets/objectnav/mp3d/v1/val/val.json.gz", "rt", encoding="utf-8"
    ) as f:
        val_data = json.load(f)
    category_to_coco = val_data.get("category_to_mp3d_category_id", {})
    id_to_name = {
        category_to_coco[cat]: MP3D_ID_TO_NAME[idx]
        for idx, cat in enumerate(category_to_coco)
    }


    # 应用Habitat配置补丁
    cfg = patch_config(cfg)
    env_count = cfg.test_epi_num
    print(env_count)
    cfg_rgb_sensor = cfg.habitat.simulator.agents.main_agent.sim_sensors.rgb_sensor

    height = cfg_rgb_sensor["height"]
    width = cfg_rgb_sensor["width"]
    fusion_score = 0.3

    # 控制相关参数
    fps = 30.0  # 帧率
    time_step = 1.0 / fps  # 时间步长

    # 当LLM被禁用时，不需要LLM输出目录

    # 添加顶视图地图和碰撞检测测量
    with habitat.config.read_write(cfg):
        cfg.habitat.task.measurements.update(
            {
                "top_down_map": TopDownMapMeasurementConfig(
                    map_padding=3,  # 地图填充
                    map_resolution=256,  # 地图分辨率
                    draw_source=True,  # 绘制源点
                    draw_border=True,  # 绘制边界
                    draw_shortest_path=True,  # 绘制最短路径
                    draw_view_points=True,  # 绘制观察点
                    draw_goal_positions=True,  # 绘制目标位置
                    draw_goal_aabbs=False,  # 不绘制目标包围盒
                    fog_of_war=FogOfWarConfig(  # 战争迷雾配置
                        draw=True,  # 启用迷雾
                        visibility_dist=5.0,  # 可见距离
                        fov=79,  # 视野角度
                    ),
                ),
                "collisions": CollisionsMeasurementConfig(),  # 碰撞检测配置
            }
        )
    # 创建Habitat环境
    env = habitat.Env(cfg)
    sim = env.sim
    # 设置重力为0（2D平面导航）
    sim.set_gravity(np.array([0.0, 0.0, 0.0]))
    # 初始化速度控制器
    vel_control = habitat_sim.physics.VelocityControl()
    vel_control.controlling_lin_vel = True  # 控制线速度
    vel_control.controlling_ang_vel = True  # 控制角速度
    vel_control.lin_vel_is_local = True  # 线速度为本地坐标系
    vel_control.ang_vel_is_local = True  # 角速度为本地坐标系

    print("Environment creation successful")

    # 跳过指定数量的episode
    while env_count:
        env.current_episode = next(env.episode_iterator)
        env_count -= 1

    # 重置环境并获取初始观察
    observations = env.reset()
    observations["rgb"] = transform_rgb_bgr(observations["rgb"])

    agent = sim.agents[0]

    # 获取初始指标
    info = env.get_metrics()
    frame = observations_to_image(observations, info)
    # cv2.imshow("Observations", frame)  # 可选：显示观察窗口

    # 初始化相机俯仰角和速度信息
    camera_pitch = 0.0
    observations["camera_pitch"] = camera_pitch
    observations["linear_velocity"] = 0.0
    observations["angular_velocity"] = 0.0
    msg_observations = deepcopy(observations)

    # 初始化ROS发布器和订阅器
    ros_pub = habitat_publisher.ROSPublisher()
    cmd_sub = rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback, queue_size=10)
    timer = rospy.Timer(rospy.Duration(0.1), publish_observations)
    itm_score_pub = rospy.Publisher("/blip2/cosine_score", Float64, queue_size=10)
    # clouds-with-scores发布器已移除（此脚本中未使用）
    confidence_threshold_pub = rospy.Publisher(
        "/detector/confidence_threshold", Float64, queue_size=10
    )
    # 发布目标标签，以便其他节点可以订阅
    label_pub = rospy.Publisher("/detector/label", String, queue_size=1, latch=True)

    print("Agent stepping around inside environment.")

    # 获取当前episode的目标对象类别
    label = env.current_episode.object_category

    # 将类别ID转换为可读名称
    if label in category_to_coco:
        coco_id = category_to_coco[label]
        label = id_to_name.get(coco_id, label)

    # 发布选定的标签，以便外部节点（例如真实世界节点）可以接收
    try:
        label_pub.publish(String(data=label))
        rospy.loginfo("Published target label: %s", label)
    except Exception as e:
        print(f"Failed to publish label: {e}")

    # 设置控制循环频率
    rate = rospy.Rate(fps)

    # 主控制循环
    tmp_cnt = 0
    while not rospy.is_shutdown() and not env.episode_over:
        loop_begin_time = rospy.Time.now()  # 记录循环开始时间

        # 初始化对象掩码（当前未使用）
        object_mask = np.zeros((height, width), dtype=np.uint8)

        # 重置速度控制
        vel_control.linear_velocity = np.array([0.0, 0.0, 0.0])  # y+ 无 x-
        vel_control.angular_velocity = np.array([0.0, 0.0, 0.0])
        timer.shutdown()

        # 设置线速度和角速度（基于接收的命令）
        vel_control.linear_velocity = np.array([0.0, 0.0, -cmd_vel])  # 负号调整方向
        vel_control.angular_velocity = np.array([0.0, cmd_omega, 0.0])

        tmp_cnt += 1
        # 在前4秒内自动旋转90度（初始扫描）
        if tmp_cnt >= 1 and tmp_cnt <= 4.0 * fps + 5:
            vel_control.angular_velocity = np.array([0.0, np.pi / 2.0, 0.0])

        # 获取当前代理状态
        agent_state = agent.state
        previous_rigid_state = habitat_sim.RigidState(
            utils.quat_to_magnum(agent_state.rotation), agent_state.position
        )

        # 计算目标刚体状态（积分变换）
        target_rigid_state = vel_control.integrate_transform(
            time_step, previous_rigid_state
        )

        # 应用步进过滤器（避免碰撞）
        end_pos = sim.step_filter(
            previous_rigid_state.translation, target_rigid_state.translation
        )

        # 更新代理状态
        agent_state.position = end_pos
        agent_state.rotation = utils.quat_from_magnum(target_rigid_state.rotation)
        agent.set_state(agent_state)

        # 定期记录寻找目标的信息
        rospy.loginfo_throttle(5.0, f"I'm finding {label}")

        # 执行环境步进（前进动作）
        observations = env.step(HabitatSimActions.move_forward)

        # 计算环境步进时间
        habitat_env_time = rospy.Time.now() - loop_begin_time

        # 获取环境指标
        info = env.get_metrics()

        # 更新观察数据
        observations["camera_pitch"] = camera_pitch
        observations["linear_velocity"] = cmd_vel
        observations["angular_velocity"] = cmd_omega

        # 发布观察数据到ROS
        ros_pub.habitat_publish_ros_topic(observations)
        msg = Float64()
        msg.data = 0.5  # 固定的置信度阈值
        confidence_threshold_pub.publish(msg)

        # 转换图像格式并清理临时数据
        observations["rgb"] = transform_rgb_bgr(observations["rgb"])
        del observations["camera_pitch"]
        del observations["linear_velocity"]
        del observations["angular_velocity"]

        # 生成可视化帧
        frame = observations_to_image(observations, info)

        # 检查是否超过时间步长（性能监控）
        if habitat_env_time.to_sec() >= time_step:
            print(
                f"env step time: {habitat_env_time.to_sec()*1000.0:.1f}ms VS {time_step*1000.0:.1f}ms"
            )

        # 控制循环频率
        rate.sleep()

    # 关闭环境
    env.close()


if __name__ == "__main__":
    # 设置信号处理器以优雅地处理中断
    signal.signal(signal.SIGINT, signal_handler)

    # 初始化ROS节点
    rospy.init_node("habitat_ros_publisher", anonymous=True)

    try:
        # 运行主函数
        main()
    except Exception as e:
        print(f"Unexpected error occurred: {e}")
        # 发生错误时关闭ROS节点
        rospy.signal_shutdown("Shutdown due to error")
        os._exit(1)
