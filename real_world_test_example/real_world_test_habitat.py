#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
真实世界机器人对象检测与导航系统 (Real World Object Detection and Navigation System)

这个脚本实现了一个完整的端到端机器人自主导航解决方案，能够在真实世界中
寻找和定位特定目标对象。通过多模态传感器融合（RGB+深度+姿态）和AI决策，
实现从自然语言指令到物理行动的完整闭环控制。

核心功能：
- 多模态传感器数据同步处理（RGB图像、深度图像、位置姿态）
- 基于YOLO和GroundingDINO的目标检测与定位
- 图像-文本匹配（ITM）语义评估
- 3D点云生成与置信度融合
- ROS通信接口，支持实时数据流

系统架构：
真实传感器 → ROS通信 → 数据同步 → AI检测/评估 → 导航决策 → 运动控制

技术特点：
- 多检测器融合：YOLO + GroundingDINO双重检测确保准确性
- 语义增强：LLM提供上下文理解和智能决策
- 实时性能：时间同步和并发控制确保实时性
- 鲁棒性：异常处理和数据验证提高系统稳定性
- 可扩展性：模块化设计支持不同传感器和算法

作者: [未指定]
日期: 2026年3月10日
"""

import os
import sys
import rospy
import numpy as np
import time
from cv_bridge import CvBridge
import message_filters
import tf.transformations as tft

import hydra
from omegaconf import DictConfig

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from plan_env.msg import MultipleMasksWithConfidence

# 动态路径配置：确保可以导入父目录的模块
current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

# 核心功能模块导入
from vlm.utils.get_object_utils import get_object  # 目标检测工具
from vlm.utils.get_itm_message import get_itm_message_cosine  # 图像-文本匹配
from llm.answer_reader.answer_reader import read_answer  # LLM答案读取器
from basic_utils.object_point_cloud_utils.object_point_cloud import (
    get_object_point_cloud,  # 3D点云生成工具
)


def inverse_habitat_publisher_transform(sensor_pose_msg):
    """
    ROS坐标系到Habitat坐标系的逆变换

    这个函数将ROS标准坐标系下的传感器姿态转换回Habitat模拟器使用的坐标系。
    这是因为ROS和Habitat使用不同的坐标系约定，需要进行坐标变换来保持一致性。

    ROS坐标系 (右手系):
    - X: 前向 (前进方向)
    - Y: 左向 (左侧方向)
    - Z: 向上 (垂直向上)

    Habitat坐标系 (右手系，但轴向不同):
    - X: 右向 (右侧方向)
    - Y: 向下 (垂直向下)
    - Z: 前向 (前进方向)

    坐标变换原理：
    - 位置变换: ROS[x, y, z] → Habitat[-y, z-camera_height, -x]
    - 姿态变换: 四元数 → 欧拉角 → 罗盘角度（弧度制）

    参数:
        sensor_pose_msg: ROS Odometry消息，包含位置和姿态信息
                        - pose.pose.position: 位置坐标 (x, y, z)
                        - pose.pose.orientation: 姿态四元数 (x, y, z, w)

    返回:
        tuple: (gps, compass)
        - gps: Habitat格式的GPS位置 [x, y, z] (numpy数组, float32)
        - compass: Habitat格式的朝向角度 [角度] (numpy数组, float32)

    变换细节：
    1. 位置变换：ROS坐标系 → Habitat坐标系
       - x_ros, y_ros, z_ros → -y_ros, z_ros-0.88, -x_ros
       - 0.88是相机高度偏移（机器人基座到相机的高度差）

    2. 姿态变换：四元数 → 欧拉角 → 罗盘角度
       - 四元数转欧拉角（ZYX顺序）
       - 提取偏航角（yaw）作为罗盘角度
       - 加上π/2补偿角度（90度），对齐坐标系差异
    """
    pos = sensor_pose_msg.pose.pose.position
    orn = sensor_pose_msg.pose.pose.orientation

    # 位置坐标逆变换：ROS → Habitat
    # ROS坐标: [x, y, z] → Habitat坐标: [-y, z-camera_height, -x]
    # camera_height = 0.88米（相机相对于地面的高度）
    gps = np.array([-pos.y, pos.z - 0.88, -pos.x], dtype=np.float32)

    # 姿态逆变换：四元数 → 欧拉角 → 罗盘角度
    # tft.euler_from_quaternion返回[roll, pitch, yaw]（弧度制）
    euler = tft.euler_from_quaternion([orn.x, orn.y, orn.z, orn.w])
    # 提取偏航角（yaw）并加上90度补偿
    compass_scalar = euler[2] + np.pi / 2.0
    # Habitat罗盘是单元素数组
    compass = np.array([compass_scalar], dtype=np.float32)

    return gps, compass


class RealWorldNode:
    """
    真实世界节点类：实现多模态传感器融合和AI决策的ROS节点

    这个类是整个系统的核心，负责：
    1. 管理ROS通信接口（发布者和订阅者）
    2. 同步处理多模态传感器数据（RGB+深度+姿态）
    3. 执行目标检测和语义评估
    4. 生成3D点云和置信度信息
    5. 与Habitat模拟器和外部导航系统进行数据交换

    工作流程：
    接收传感器数据 → 数据同步 → 目标检测 → 语义评估 → 结果发布

    并发控制机制：
    - 使用processing_detect和processing_value标志防止任务重叠
    - 确保计算资源不会被过度占用
    - 自适应调整处理频率

    异常处理策略：
    - 捕获并记录所有异常，不中断主程序运行
    - 提供有意义的错误信息便于调试
    - 优雅降级：某些组件失败时仍能继续运行
    """

    def __init__(self, cfg):
        """
        初始化真实世界节点

        设置ROS通信接口、数据同步器、AI模型配置等核心组件。
        建立完整的传感器数据处理管道，从原始数据到智能决策。

        参数:
            cfg: Hydra配置对象，包含检测器、LLM、传感器等配置信息
                - detector: 检测器配置（YOLO、GroundingDINO参数）
                - llm: 大语言模型配置
                - habitat_sensor: 传感器参数
        """
        self.config = cfg

        # 初始化ROS节点
        # anonymous=False 表示节点名称唯一，不允许重名
        # 这确保了在ROS网络中只有一个该类型的节点实例
        rospy.init_node("real_world_node", anonymous=False)

        # OpenCV-ROS图像转换桥接器
        # 用于在ROS Image消息和OpenCV图像格式之间转换
        self.bridge = CvBridge()

        # ===== ROS订阅者配置 =====
        # 使用message_filters实现多话题时间同步订阅
        # 确保RGB、深度、姿态数据的时间一致性

        # RGB图像订阅器：订阅相机RGB图像流
        self.rgb_sub_ = message_filters.Subscriber("/habitat/camera_rgb", Image)

        # 深度图像订阅器：订阅相机深度图像流
        self.depth_sub_ = message_filters.Subscriber("/habitat/camera_depth", Image)

        # 传感器姿态订阅器：订阅传感器姿态信息，用于坐标变换
        self.sensor_pose_sub_ = message_filters.Subscriber(
            "/habitat/sensor_pose", Odometry
        )

        # 里程计订阅器：订阅机器人里程计数据，用于位置跟踪  看起来根本没用上
        # queue_size=10 表示消息队列长度，防止消息积压
        rospy.Subscriber("/habitat/odom", Odometry, self.odom_callback, queue_size=10)

        # ===== ROS发布者配置 =====

        # 检测置信度阈值发布器：告诉检测器使用什么置信度阈值
        # 这个阈值用于过滤低置信度的检测结果
        self.confidence_threshold_pub_ = rospy.Publisher(
            "/detector/confidence_threshold", Float64, queue_size=10
        )

        # 图像-文本匹配分数发布器：发布语义相似度评估结果
        # 用于评估当前图像与目标描述的相关性
        self.itm_score_pub_ = rospy.Publisher(
            "/blip2/cosine_score", Float64, queue_size=10
        )

        # 带置信度的3D点云发布器：发布检测结果的3D表示
        # 包含每个检测到的对象的点云数据和置信度分数
        self.cld_with_score_pub_ = rospy.Publisher(
            "/detector/clouds_with_scores", MultipleMasksWithConfidence, queue_size=10
        )

        # 检测结果可视化图像发布器：发布标注了检测结果的图像
        # 用于调试和可视化检测效果
        self.detect_img_pub_ = rospy.Publisher(
            "/detector/detect_img", Image, queue_size=10
        )

        # ===== 数据同步器配置 =====

        # 目标检测同步器：同步RGB、深度、姿态数据用于目标检测
        # 当三者时间戳相近（10ms内）时触发检测回调
        self.sync_detect = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub_, self.depth_sub_, self.sensor_pose_sub_],
            queue_size=5,      # 缓冲队列大小，存储等待同步的消息
            slop=0.01,         # 时间同步容差：10ms (0.01秒)
        )
        # 注册检测回调函数
        self.sync_detect.registerCallback(self.sync_detect_callback)

        # 价值评估同步器：同步数据用于语义相似度计算
        # 使用相同的同步参数确保一致性
        self.sync_value = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub_, self.depth_sub_, self.sensor_pose_sub_],
            queue_size=5,
            slop=0.01,
        )
        # 注册评估回调函数
        self.sync_value.registerCallback(self.sync_value_callback)

        # ===== 状态变量初始化 =====

        # 里程计数据存储：保存最新的机器人位置和姿态信息
        self.robot_odom = None
        self.T_base_camera = None  # 基座到相机的变换矩阵（预留）
        self.odom_stamp = None

        # 并发处理控制标志：防止多个处理任务同时运行
        # 确保计算资源不会被过度占用，自适应调整处理频率
        self.processing_detect = False  # 目标检测处理中标志
        self.processing_value = False   # 价值评估处理中标志

        # ===== LLM配置（大语言模型增强）=====
        # LLM用于提供上下文理解和智能决策支持
        llm_cfg = self.config.llm
        self.llm_answer_path = llm_cfg.llm_answer_path      # LLM答案文件路径
        self.llm_response_path = llm_cfg.llm_response_path  # LLM响应文件路径
        self.llm_client = llm_cfg.llm_client.llm_client     # LLM客户端类型

        # ===== 目标标签和LLM答案初始化 =====
        # 目标标签通过ROS话题 /detector/label 接收（来自Habitat模拟器）
        # 初始值设为空，实际值在label_callback中设置
        self.label = None         # 当前目标标签（如"chair", "table"）
        self.llm_answer = []      # LLM提供的答案列表（增强检测提示）
        self.room = None          # 当前房间信息（用于语义上下文）
        self.fusion_score = 0.0   # 融合置信度分数（LLM决策结果）

        # ===== ROS话题订阅 =====
        # 订阅目标标签话题，当Habitat发布新目标时会触发回调
        # queue_size=1 表示只保留最新的标签消息
        rospy.Subscriber("/detector/label", String, self.label_callback, queue_size=1)

        # ===== 定时器配置 =====
        # 每秒发布一次置信度阈值，确保检测器始终知道当前阈值
        # 这对于动态调整检测灵敏度很重要
        rospy.Timer(rospy.Duration(1.0), self.publish_confidence_threshold)

    def sync_detect_callback(self, rgb_msg, depth_msg, sensor_pose_msg):
        """
        目标检测同步回调函数

        当RGB图像、深度图像和传感器姿态数据同步到达时触发此函数。
        执行完整的对象检测、定位和3D点云生成流程。

        处理流程：
        1. 并发控制检查（防止多任务同时运行）
        2. 时间戳同步验证（确保数据时间一致性）
        3. 图像数据预处理（ROS消息 → OpenCV格式）
        4. 多检测器目标检测（YOLO + GroundingDINO + LLM增强）
        5. 坐标系变换（ROS → Habitat坐标系）
        6. 3D点云生成（2D检测结果投影到3D空间）
        7. 结果发布（点云、图像、可视化数据）

        技术细节：
        - 时间同步容差：100ms（超过此值的数据对将被丢弃）
        - 检测器融合：结合传统检测器和开放词汇检测器的优势
        - 3D重建：使用深度信息将2D边界框转换为3D点云
        - 置信度融合：多源检测结果的置信度加权融合

        参数:
            rgb_msg: ROS Image消息，RGB图像数据
                    - 格式：sensor_msgs/Image
                    - 编码：rgb8 (24位RGB)
            depth_msg: ROS Image消息，深度图像数据
                     - 格式：sensor_msgs/Image
                     - 编码：32FC1 (32位浮点单通道)
            sensor_pose_msg: ROS Odometry消息，传感器姿态数据
                           - 包含位置和姿态信息
                           - 用于坐标系变换

        异常处理：
        - 捕获所有异常并记录错误日志
        - 不中断程序运行，确保系统稳定性
        - 在finally块中重置处理标志
        """
        # 如果检测任务正在进行，跳过此次调用
        if self.processing_detect:
            return
        self.processing_detect = True
        try:
            # 时间戳同步验证  应该是老代码，现在没有用
            stamp = rgb_msg.header.stamp
            time_diff = abs((stamp - sensor_pose_msg.header.stamp).to_sec())
            if time_diff > 0.1:
                # 如果时间戳差异超过100ms，跳过此帧（数据不同步）
                return

            # 图像数据预处理
            rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
            depth_img = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )
            #深度图显式转换成float32
            transform_depth_img = depth_img.astype(np.float32)
            # 2维变三维
            depth_cv = np.expand_dims(transform_depth_img, axis=-1)

            # 初始化结果消息
            cld_with_score_msg = MultipleMasksWithConfidence()
            cld_with_score_msg.point_clouds = []
            cld_with_score_msg.confidence_scores = []
            cld_with_score_msg.label_indices = []
            rospy.loginfo("detect: label: %s", self.label)

            # 如果标签还未收到，跳过检测直到可用
            if self.label is None:
                rospy.logwarn_throttle(5.0, "Waiting for target label on /detector/label")
                return

            # 执行目标检测
            detect_img, score_list, object_masks_list, label_list = get_object(
                self.label, rgb_cv, self.config.detector, self.llm_answer
            )

            # 使用逆变换恢复原始Habitat观察格式
        # - gps: Habitat格式的GPS位置 [x, y, z] (numpy数组, float32)
        # - compass: Habitat格式的朝向角度 [角度] (numpy数组, float32)
            gps, compass = inverse_habitat_publisher_transform(sensor_pose_msg)

            observations = {
                "depth": depth_cv,
                "gps": gps,
                "compass": compass,  # 已经是numpy数组
            }

            # 生成对象点云
            obj_point_cloud_list = get_object_point_cloud(
                self.config, observations, object_masks_list
            )
            cld_with_score_msg.point_clouds = obj_point_cloud_list
            cld_with_score_msg.confidence_scores = score_list
            cld_with_score_msg.label_indices = label_list

            # 发布检测图像以进行可视化
            self.detect_img_pub_.publish(
                self.bridge.cv2_to_imgmsg(detect_img, encoding="rgb8")
            )

            # 也发布检测到的对象云及其分数，以便其他节点/RViz可以使用它们
            self.cld_with_score_pub_.publish(cld_with_score_msg)
        except Exception as e:
            rospy.logerr("detect: Error in synchronized processing: %s", e)
        finally:
            # 标记处理完成，以便下一次调用可以进行
            self.processing_detect = False

#对当前同步到的一帧 RGB 图像，计算它和目标文本的语义相似度，并把这个分数发布给规划侧的 ValueMap。
    def sync_value_callback(self, rgb_msg, depth_msg, sensor_pose_msg):
        """
        价值评估同步回调函数

        计算当前图像与目标对象的语义相似度（Image-Text Matching）。
        使用深度学习模型评估图像内容与目标描述的相关性，为导航决策提供语义指导。

        处理流程：
        1. 并发控制检查
        2. 时间戳同步验证
        3. 图像预处理
        4. 语义相似度计算（ITM分数）
        5. 结果发布

        技术细节：
        - 使用BLIP-2或其他视觉-语言模型
        - 计算余弦相似度作为相关性度量
        - 结合房间上下文信息提高准确性
        - 分数范围：通常在[-1, 1]之间，值越大相似度越高

        参数:
            rgb_msg: ROS Image消息，RGB图像数据
            depth_msg: ROS Image消息，深度图像数据（此函数中未使用）
            sensor_pose_msg: ROS Odometry消息，传感器姿态数据（此函数中未使用）

        输出:
        - 发布到 /blip2/cosine_score 话题
        - Float64类型的相似度分数
        """
        # 如果评估任务正在进行，跳过此次调用
        if self.processing_value:
            return
        self.processing_value = True
        try:
            # 时间戳同步验证
            stamp = rgb_msg.header.stamp
            time_diff = abs((stamp - sensor_pose_msg.header.stamp).to_sec())
            if time_diff > 0.1:
                # 如果时间戳差异显著，跳过此对
                return

            # 图像预处理
            rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")

            # 计算图像-文本匹配的余弦相似度
            cosine = get_itm_message_cosine(rgb_cv, self.label, self.room)
            rospy.loginfo("value: Computed cosine score: %.3f", cosine)

            # 封装并发布ITM分数
            # 创建一个 std_msgs/Float64 类型的消息实例
            itm_score_msg = Float64()
            itm_score_msg.data = cosine
            self.itm_score_pub_.publish(itm_score_msg)
        except Exception as e:
            # 记录错误但不中断程序
            rospy.logerr("value: Error in synchronized processing: %s", e)
        finally:
            # 标记评估任务完成
            self.processing_value = False

    def label_callback(self, msg):
        """
        目标标签回调函数

        当接收到新的目标标签时触发，更新系统状态并获取LLM增强信息。
        这是系统接收任务指令的关键入口点。

        处理流程：
        1. 检查标签是否变化（避免重复处理）
        2. 更新当前标签
        3. 从LLM获取增强答案（如果配置了LLM）
        4. 更新房间上下文和融合分数

        LLM增强作用：
        - 提供目标的语义描述和上下文信息
        - 帮助检测器更好地理解目标特征
        - 提供房间级别的环境理解

        参数:
            msg: ROS String消息，包含目标标签（如"chair", "table"）

        异常处理：
        - LLM调用失败时不中断程序，使用默认值
        - 记录警告信息便于调试
        """
        try:
            # 提取新的目标标签
            new_label = str(msg.data)

            # 如果标签没有变化，跳过处理
            if new_label == self.label:
                return

            # 更新当前标签
            self.label = new_label
            rospy.loginfo("Received target label: %s", self.label)

            # 如果LLM已配置，获取增强答案
            try:
                # 从LLM获取目标的详细描述、房间信息和融合阈值 这个融合阈值似乎没用上
                self.llm_answer, self.room, self.fusion_score = read_answer(
                    self.llm_answer_path,    # LLM答案文件路径
                    self.llm_response_path,  # LLM响应文件路径
                    self.label,              # 当前目标标签
                    self.llm_client          # LLM客户端类型
                )
            except Exception:
                # LLM调用失败时的降级处理
                self.llm_answer = []      # 使用空答案列表
                self.room = None          # 无房间上下文
                self.fusion_score = 0.0   # 默认融合分数
        except Exception as e:
            # 标签处理异常记录
            rospy.logerr("label_callback: Error processing label message: %s", e)

    def odom_callback(self, msg):
        """
        里程计数据回调函数

        处理来自机器人的里程计信息，用于位置跟踪和状态监控。
        目前主要用于数据接收和时间戳管理。

        参数:
            msg: ROS Odometry消息，包含机器人位置和姿态

        注意：
        - 当前实现相对简单，主要用于数据同步
        - publish_sensor_pose() 调用被注释，可能在未来版本中使用
        - 时间戳管理确保数据时序正确性
        """
        try:
            # 保存最新的里程计数据
            self.robot_odom = msg
            self.odom_stamp = msg.header.stamp

            # 时间戳处理逻辑（当前被注释）
            if self.odom_stamp is not None:
                # self.publish_sensor_pose()  # 预留的传感器姿态发布功能
                self.odom_stamp = None

            # 可选：记录里程计接收日志
            # rospy.loginfo("odom: Received Odometry")
        except Exception as e:
            # 里程计处理异常记录
            rospy.logerr("odom: Error processing Odometry: %s", e)

    def publish_confidence_threshold(self, event):
        """
        置信度阈值发布函数

        定时发布检测置信度阈值，确保所有检测器使用一致的阈值标准。
        这个阈值用于过滤低置信度的检测结果，提高检测准确性。

        定时机制：
        - 由ROS Timer触发，每秒执行一次
        - 确保阈值及时更新到所有相关节点

        阈值作用：
        - 检测器使用此阈值过滤检测结果
        - 低于阈值的检测结果被丢弃
        - 平衡检测召回率和精确率

        参数:
            event: ROS Timer事件（定时触发）

        发布内容：
        - 话题：/detector/confidence_threshold
        - 类型：Float64
        - 值：0.5（固定的置信度阈值）
        """
        # 创建置信度阈值消息
        confidence_threshold_msg = Float64()
        confidence_threshold_msg.data = 0.5  # 固定的置信度阈值

        # 发布到检测器配置话题
        self.confidence_threshold_pub_.publish(confidence_threshold_msg)

    def run(self):
        """
        主运行函数

        启动ROS节点的主循环，等待传感器数据并处理回调。
        这是系统运行的入口点，阻塞执行直到节点关闭。

        运行流程：
        1. 记录启动信息
        2. 进入ROS自旋循环（阻塞等待）
        3. 处理所有订阅的回调函数
        4. 响应系统关闭信号

        注意：
        - 这个函数会阻塞直到ROS节点关闭
        - 所有实际处理都在回调函数中进行
        - 异常处理在更高级别（main函数）进行
        """
        # 记录节点启动信息
        rospy.loginfo("RealWorldNode running. Waiting for sensor messages...")

        # 进入ROS主循环，等待消息和处理回调
        # 这个调用会阻塞直到节点关闭（Ctrl+C或其他关闭信号）
        rospy.spin()

    # ===== 主程序入口 =====
#hydra传入配置文件 real_world_test.yaml  赋值给cfg
@hydra.main(version_base=None, config_path="config", config_name="real_world_test")
def main(cfg: DictConfig):
    """
    主函数：程序入口点

    使用Hydra进行配置管理，创建并运行真实世界节点。
    处理程序级别的异常，确保优雅关闭。

    执行流程：
    1. Hydra配置加载和解析
    2. RealWorldNode实例创建
    3. 节点运行（阻塞调用）
    4. 异常处理和清理

    参数:
        cfg: Hydra配置对象，包含所有系统配置
            - detector: 检测器配置
            - llm: 大语言模型配置
            - habitat_sensor: 传感器参数

    异常处理：
    - 捕获所有未处理的异常
    - 记录错误信息
    - 确保程序正常退出
    """
    try:
        # 创建真实世界节点实例
        node = RealWorldNode(cfg)

        # 启动节点运行（阻塞调用）
        node.run()

    except Exception as e:
        # 全局异常处理：记录错误但确保程序退出
        print(f"Unexpected error in main: {e}")
        # 注意：这里没有调用rospy.signal_shutdown()，
        # 因为rospy.spin()可能已经处理了关闭信号

if __name__ == "__main__":
    # 脚本直接执行时的入口点
    # 调用Hydra装饰器包装的main函数
    main()