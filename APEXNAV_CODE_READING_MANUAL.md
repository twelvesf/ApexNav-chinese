# ApexNav Code Reading Manual
我的总结:

视觉前端
vlm/detector/grounding_dino.py  
<!-- //GroundingDINO 的模型封装 + 客户端/服务端接口层 
输出：哪些位置可能有这些物体的 bounding box 检测框，以及对应分数、类别文字 -->

vlm/itm/blip2itm.py
<!-- # BLIP2：视觉语言模型   图像文本匹配打分器  
# cosine：适合“前方整体看起来像不像目标相关区域”
# itm_score：适合“这个候选框到底是不是目标”
# Client-Server工程封装  -->

segmentor/sam.py
<!-- # 输入groundingdino/yolo传来的框，输出分割mask  mobilesam：扣出框内的目标像素
# mask : 与图像同尺寸的二值区域图，用来表示哪些图像属于目标哪些不属于 -->

vlm/detector/yolov7.py
<!-- # YOLOv7：固定类别检测器，适合常见物体，快、直接 -->


主要算法:
## ApexNav/src/planner/exploration_manager/launch/exploration_traj.launch  主算法启动launch
# ApexNav/src/planner/exploration_manager/launch/algorithm_traj.xml 真正启动算法的xml文件  同时配置参数
   ApexNav/src/planner/exploration_manager/src/exploration_node.cpp  
   * 该节点是ApexNav探索系统的入口点，负责根据配置参数选择仿真模式或真实世界模式的有限状态机进行自主探索任务
   
























## 1. 这份手册解决什么问题

这份手册的目标不是介绍如何运行 `ApexNav`，而是帮助你从代码层面快速回答三个问题：

1. `ApexNav` 的主链路到底怎么跑起来。
2. 目标语义信息是怎么进入规划器的。
3. 如果以后要和 `FUEL_GAZEBO` 融合，哪些模块值得迁移，哪些模块不值得动。

如果你的最终目标是“以 `FUEL_GAZEBO` 为底座，引入 ApexNav 的视觉语言引导搜索能力”，那么这套代码里最值得精读的不是控制执行层，而是：

- `plan_env/value_map2d.*`
- `plan_env/object_map2d.*`
- `plan_env/map_ros.*`
- `exploration_manager/exploration_manager.*`
- `real_world_test_example/real_world_test_habitat.py`


## 2. 先建立整体图

先把 ApexNav 看成 5 层：

1. 传感器/模拟器桥接层  
   把 Habitat 或真实传感器数据发布成 ROS 话题。

2. 几何建图层  
   用深度图更新 2D 占据栅格和 ESDF。

3. 语义映射层  
   把检测结果、ITM 分数融合成对象地图和语义价值图。

4. 决策层  
   在 frontier、object、value map 上做目标搜索策略选择。

5. 执行层  
   仿真模式下输出离散动作；trajectory 模式下输出连续轨迹，再转成 `/cmd_vel`。

一句话理解：

- `ApexNav` 的几何世界是 `SDFMap2D + FrontierMap2D`
- `ApexNav` 的语义世界是 `ObjectMap2D + ValueMap`
- `ApexNav` 的搜索策略写在 `ExplorationManager`


## 3. 两条运行链必须分开看

### 3.1 仿真模式

入口：

- [`exploration.launch`](./src/planner/exploration_manager/launch/exploration.launch)
- [`algorithm.xml`](./src/planner/exploration_manager/launch/algorithm.xml)
- [`exploration_node.cpp`](./src/planner/exploration_manager/src/exploration_node.cpp)
- [`exploration_fsm.cpp`](./src/planner/exploration_manager/src/exploration_fsm.cpp)

这条链的特点：

- `is_real_world=false`
- 主状态机是 `ExplorationFSM`
- 输出不是连续轨迹，而是 Habitat 离散动作 `/habitat/plan_action`

如果你是为了后续和 `FUEL_GAZEBO` 融合，这条链只需要理解“它怎么做搜索决策”，不需要深入它的动作执行细节。

### 3.2 trajectory / real-world 模式

入口：

- [`exploration_traj.launch`](./src/planner/exploration_manager/launch/exploration_traj.launch)
- [`algorithm_traj.xml`](./src/planner/exploration_manager/launch/algorithm_traj.xml)
- [`exploration_node.cpp`](./src/planner/exploration_manager/src/exploration_node.cpp)
- [`exploration_fsm_traj.cpp`](./src/planner/exploration_manager/src/exploration_fsm_traj.cpp)
- [`traj_server.cpp`](./src/planner/trajectory_manager/src/traj_server.cpp)

这条链的特点：

- `is_real_world=true`
- 主状态机是 `ExplorationFSMReal`
- 规划器输出 `PolyTraj` 到 `/planning/trajectory`
- `traj_server` 把 `PolyTraj` 跟踪成 `/cmd_vel`

如果你以后底座改成 `FUEL_GAZEBO + PX4`，这条执行链大概率不会保留，但它的“语义驱动搜索决策”仍然值得读。


## 4. 推荐阅读顺序

建议按下面顺序读，不要一上来钻 `gcopter` 或 `kino_astar`。

### 第一轮：只建立主链路

1. [`README.md`](./README.md)  
   先知道项目官方定位。它明确说是 “target-centric semantic fusion”。

2. [`exploration.launch`](./src/planner/exploration_manager/launch/exploration.launch)  
   看仿真模式总入口。

3. [`exploration_traj.launch`](./src/planner/exploration_manager/launch/exploration_traj.launch)  
   看 trajectory 模式总入口。

4. [`algorithm.xml`](./src/planner/exploration_manager/launch/algorithm.xml)  
   看 planner 需要哪些 topic 和参数。

5. [`algorithm_traj.xml`](./src/planner/exploration_manager/launch/algorithm_traj.xml)  
   看 real-world 版本多了哪些参数。

6. [`exploration_node.cpp`](./src/planner/exploration_manager/src/exploration_node.cpp)  
   它只做一件事：根据 `is_real_world` 选择两个不同 FSM。

### 第二轮：搞懂地图和语义输入

7. [`sdf_map2d.cpp`](./src/planner/plan_env/src/sdf_map2d.cpp)  
   看 `initMap()` 怎么把 `MapROS`、`ObjectMap2D`、`ValueMap` 串起来。

8. [`map_ros.h`](./src/planner/plan_env/include/plan_env/map_ros.h)  
   看 ROS 侧订阅/发布了什么。

9. [`map_ros.cpp`](./src/planner/plan_env/src/map_ros.cpp)  
   看深度图、对象点云、ITM 分数分别怎么进入地图。

10. [`frontier_map2d.cpp`](./src/planner/plan_env/src/frontier_map2d.cpp)  
    看几何 frontier 是怎么提取的。

11. [`value_map2d.h`](./src/planner/plan_env/include/plan_env/value_map2d.h)
12. [`value_map2d.cpp`](./src/planner/plan_env/src/value_map2d.cpp)  
    这是 ApexNav 最值得迁移到 FUEL 的模块之一。

13. [`object_map2d.h`](./src/planner/plan_env/include/plan_env/object_map2d.h)
14. [`object_map2d.cpp`](./src/planner/plan_env/src/object_map2d.cpp)  
    这是 ApexNav 另一块核心：多次检测融合、对象置信度演化、对象云维护。

### 第三轮：搞懂搜索策略

15. [`exploration_manager.h`](./src/planner/exploration_manager/include/exploration_manager/exploration_manager.h)
16. [`exploration_manager.cpp`](./src/planner/exploration_manager/src/exploration_manager.cpp)

这两份文件定义了 ApexNav 真正的策略层：

- 什么时候追高置信度对象
- 什么时候继续 frontier exploration
- 什么时候用 semantic frontier
- 什么时候用 hybrid
- 什么时候切到 TSP

### 第四轮：最后再看执行层

17. [`exploration_fsm.cpp`](./src/planner/exploration_manager/src/exploration_fsm.cpp)  
    仿真下的离散动作 FSM。

18. [`exploration_fsm_traj.cpp`](./src/planner/exploration_manager/src/exploration_fsm_traj.cpp)  
    trajectory 模式下的连续轨迹 FSM。

19. [`kino_astar.cpp`](./src/planner/path_searching/src/kino_astar.cpp)
20. [`trajectory_manager/optimizer.h`](./src/planner/trajectory_manager/include/trajectory_manager/optimizer.h)
21. [`traj_server.cpp`](./src/planner/trajectory_manager/src/traj_server.cpp)

如果你的目标是和 `FUEL_GAZEBO` 融合，这一轮可以最后再看。


## 5. 先记住这几个主入口

### 5.1 ROS 主入口

- [`exploration_node.cpp`](./src/planner/exploration_manager/src/exploration_node.cpp)

逻辑很简单：

- `is_real_world=false` -> `ExplorationFSM`
- `is_real_world=true` -> `ExplorationFSMReal`

这说明 ApexNav 实际上维护了两套执行范式，但共用同一套地图和搜索策略核心。

### 5.2 地图入口

- [`SDFMap2D::initMap`](./src/planner/plan_env/src/sdf_map2d.cpp)

这一步会初始化：

- `ObjectMap2D`
- `ValueMap`
- `MapROS`

因此你可以把 `SDFMap2D` 看成整个 planner 世界模型的根节点。

### 5.3 搜索决策入口

- [`ExplorationManager::planNextBestPoint`](./src/planner/exploration_manager/src/exploration_manager.cpp)

以后你如果只想定位 “ApexNav 到底在哪决定往哪走”，就直接从这里读。


## 6. 主数据流怎么走

下面是最值得记住的一条链：

### 6.1 几何地图链

`depth + sensor_pose -> MapROS::depthPoseCallback -> processDepthImage -> filterPointCloudToXY -> SDFMap2D::inputDepthCloud2D`

对应文件：

- [`map_ros.cpp`](./src/planner/plan_env/src/map_ros.cpp)
- [`sdf_map2d.cpp`](./src/planner/plan_env/src/sdf_map2d.cpp)

结果：

- 生成 2D 占据地图
- 更新 free / occupied / unknown
- 更新局部 ESDF

### 6.2 语义地图链

这里原来容易写错的地方是：`ApexNav` 的“语义地图”不是一张图，而是两种并行表征：

1. `ObjectMap2D`：对象级语义实体图  
   回答“哪里有目标/相似物对象，置信度如何”
2. `ValueMap`：连续空间语义价值图  
   回答“当前自由空间里，哪里在语义上更像目标相关区域”

按你实际测试时的运行方式，这条链路应当这样理解。

#### 前端输入是谁产生的

运行：

- `python -m vlm.detector.grounding_dino --port 12181`
- `python -m vlm.itm.blip2itm --port 12182`
- `python -m vlm.segmentor.sam --port 12183`
- `python -m vlm.detector.yolov7 --port 12184`
- `python habitat_vel_control.py`
- `python ./real_world_test_example/real_world_test_habitat.py`

其中：

- [`habitat_vel_control.py`](./habitat_vel_control.py) 通过 [`habitat_publisher.py`](./habitat2ros/habitat_publisher.py) 发布
  - `/habitat/camera_rgb`
  - `/habitat/camera_depth`
  - `/habitat/sensor_pose`
  - `/habitat/odom`
  - `/detector/label`
- [`real_world_test_habitat.py`](./real_world_test_example/real_world_test_habitat.py) 是真正的语义前端
  - `sync_detect_callback()`：
    `RGB + depth + sensor_pose -> get_object(YOLOv7/GroundingDINO + SAM) -> get_object_point_cloud -> /detector/clouds_with_scores`
  - `sync_value_callback()`：
    `RGB -> get_itm_message_cosine(BLIP2-ITM cosine) -> /blip2/cosine_score`

#### `ObjectMap2D` 是怎么形成的

对象级语义链路是：

`/detector/clouds_with_scores -> MapROS::detectedObjectCloudCallback -> inputObjectCloud2D -> ObjectMap2D`

这里不是简单“收到点云就存起来”，中间还做了：

- ROS `PointCloud2` 转 PCL
- voxel downsample
- 超量程点过滤
- DBSCAN 去噪
- `label == 0` 的 over-depth object 维护
- `ObjectMap2D` 内部对象聚类与多次观测融合

所以 `ObjectMap2D` 形成的是：

- 离散对象簇
- 每个对象的 `best_label`
- 每个标签的 `confidence_scores_`
- `good_cells_`
- `over_depth_object_cloud_`

#### `ValueMap` 是怎么形成的

这里最容易误解。

不是：

`/blip2/cosine_score -> itmScoreCallback -> 直接更新 ValueMap`

实际代码是两步：

1. [`MapROS::itmScoreCallback`](./src/planner/plan_env/src/map_ros.cpp)  
   只把最新的 `itm_score_` 缓存在 `MapROS` 里
2. [`MapROS::depthPoseCallback`](./src/planner/plan_env/src/map_ros.cpp)  
   在处理当前深度图、得到当前视场覆盖的 `free_grids` 之后，才调用  
   `ValueMap::updateValueMap(camera_pos, camera_yaw, free_grids, itm_score_)`

也就是说，`ValueMap` 的形成逻辑是：

`depth + sensor_pose -> 当前视场 free_grids`

`最新缓存的 itm_score + 当前 camera_pos/camera_yaw + free_grids -> ValueMap::updateValueMap`

这意味着：

- `ValueMap` 不是由目标检测框直接生成的
- `itmScoreCallback()` 只是缓存分数，不负责落图
- 真正的语义落图发生在 `depthPoseCallback()` 里
- 语义值只会投到“当前视场里可见的自由空间格子”上，而不是投到 object cells 上

[`value_map2d.cpp`](./src/planner/plan_env/src/value_map2d.cpp) 里具体做的是：

- 对每个 `free_grid`
- 根据它相对相机光轴的夹角计算一个 `FOV confidence`
- 用 `itm_score` 和历史 `value_buffer_ / confidence_buffer_` 做加权融合

所以 `ValueMap` 的本质不是“对象语义图”，而是：

**一张由视场观测不断累积得到的、连续空间上的语义价值图。**

#### 这两张语义地图最后怎么被使用

在 [`ExplorationManager::planNextBestPoint`](./src/planner/exploration_manager/src/exploration_manager.cpp) 里，两者分工完全不同：

- `ObjectMap2D`
  - 先被用来取高置信目标对象
  - 决定是否进入 `SEARCH_BEST_OBJECT / SEARCH_OVER_DEPTH_OBJECT / SEARCH_SUSPICIOUS_OBJECT`
  - 更像“对象驱动接近”

- `ValueMap`
  - 不直接触发追对象
  - 而是在 `getSortedSemanticFrontiers()`、`findHighestSemanticsFrontierPolicy()`、`hybridExplorePolicy()` 里
    通过 `value_map_->getValue(...)` 给 frontier 附近区域打语义分
  - 更像“语义驱动 frontier 排序”

所以最终决策逻辑不是“先做一张统一语义图再全都用同一种方式消费”，而是：

- `ObjectMap2D` 负责对象级确认与接近
- `ValueMap` 负责空间级语义偏好
- `ExplorationManager` 把两者和 `FrontierMap2D` 一起融合成下一步搜索决策

对应文件：

- [`real_world_test_habitat.py`](./real_world_test_example/real_world_test_habitat.py)
- [`habitat_vel_control.py`](./habitat_vel_control.py)
- [`habitat_publisher.py`](./habitat2ros/habitat_publisher.py)
- [`map_ros.cpp`](./src/planner/plan_env/src/map_ros.cpp)
- [`object_map2d.cpp`](./src/planner/plan_env/src/object_map2d.cpp)
- [`value_map2d.cpp`](./src/planner/plan_env/src/value_map2d.cpp)

一句话总结这一段：

`ApexNav` 的语义输入不是“检测结果直接变成语义地图”，而是分成两路：

- 检测点云进入 `ObjectMap2D`
- ITM 分数结合深度视场进入 `ValueMap`

然后再由 `ExplorationManager` 以“对象优先、frontier 次之、语义排序辅助”的方式统一消费。

### 6.3 决策链

`SDFMap2D + FrontierMap2D + ObjectMap2D + ValueMap -> ExplorationManager::planNextBestPoint`

策略优先级大致是：

1. 先看高置信度对象能不能直接去
2. 再看 over-depth object
3. 再做 frontier exploration
4. 如果 frontier 不通，再回退到可疑对象或 extreme mode

这条逻辑写在：

- [`exploration_manager.cpp`](./src/planner/exploration_manager/src/exploration_manager.cpp)


## 7. 哪些类最重要

### 7.1 `SDFMap2D`

文件：

- [`sdf_map2d.h`](./src/planner/plan_env/include/plan_env/sdf_map2d.h)
- [`sdf_map2d.cpp`](./src/planner/plan_env/src/sdf_map2d.cpp)

职责：

- 2D 占据栅格
- ESDF
- 射线投影更新
- 碰撞与安全性查询

你可以把它看成 ApexNav 的“几何底盘”。

### 7.2 `MapROS`

文件：

- [`map_ros.h`](./src/planner/plan_env/include/plan_env/map_ros.h)
- [`map_ros.cpp`](./src/planner/plan_env/src/map_ros.cpp)

职责：

- 订阅深度图和位姿
- 把深度投影成地图
- 接收对象点云和 ITM 分数
- 调用 `ObjectMap2D` 与 `ValueMap`

这是整个系统最重要的 ROS 适配层。

### 7.3 `FrontierMap2D`

文件：

- [`frontier_map2d.h`](./src/planner/plan_env/include/plan_env/frontier_map2d.h)
- [`frontier_map2d.cpp`](./src/planner/plan_env/src/frontier_map2d.cpp)

职责：

- 在 2D 栅格上提取 frontier
- 聚类
- PCA 分裂大 frontier

这部分是几何探索基础，不包含语言语义。

### 7.4 `ValueMap`

文件：

- [`value_map2d.h`](./src/planner/plan_env/include/plan_env/value_map2d.h)
- [`value_map2d.cpp`](./src/planner/plan_env/src/value_map2d.cpp)

职责：

- 用 ITM 分数更新空间语义价值
- 维护每个栅格的 `value_buffer_` 和 `confidence_buffer_`
- 用 FOV 置信模型把单帧语义分数投射到空间

这是 ApexNav 里和你未来 FUEL 融合最直接相关的模块。

核心思想：

- 不是给某个 frontier 直接打分
- 而是先维护一张连续的语义价值图
- 再由搜索策略去消费这张图

### 7.5 `ObjectMap2D`

文件：

- [`object_map2d.h`](./src/planner/plan_env/include/plan_env/object_map2d.h)
- [`object_map2d.cpp`](./src/planner/plan_env/src/object_map2d.cpp)

职责：

- 聚合多次检测得到的对象点云
- 维护对象类别置信度
- 维护 over-depth object
- 支持“看到”和“没看到”的双向融合

这是 ApexNav 里第二块最值得迁移的模块。

### 7.6 `ExplorationManager`

文件：

- [`exploration_manager.h`](./src/planner/exploration_manager/include/exploration_manager/exploration_manager.h)
- [`exploration_manager.cpp`](./src/planner/exploration_manager/src/exploration_manager.cpp)

职责：

- 汇总 frontier / object / value map
- 选择探索策略
- 输出 `next_pos_` 和 `next_best_path_`
- trajectory 模式下继续调用 `KinoAstar + GCopter`

如果你以后要把 ApexNav 的“搜索策略”嫁接到 FUEL，这个类是第一阅读重点。


## 8. `ExplorationManager` 到底做了什么

建议重点看 `planNextBestPoint()`。

### 8.1 逻辑优先级

代码不是“永远按 frontier 搜”，而是分层决策：

1. 高置信度目标对象优先
2. over-depth object 次之
3. 普通 frontier exploration
4. 可疑对象/极端搜索兜底

这说明 ApexNav 的核心不是“semantic frontier only”，而是：

- object-driven navigation
- frontier-driven exploration
- semantic value driven frontier ranking

三者混合。

### 8.2 它有几种探索策略

`policy_mode` 定义在 launch 参数里：

- `0`: distance
- `1`: semantic
- `2`: hybrid
- `3`: TSP(distance)

对应实现都在 [`exploration_manager.cpp`](./src/planner/exploration_manager/src/exploration_manager.cpp)。

其中最值得你关注的是：

- `findHighestSemanticsFrontierPolicy()`
- `hybridExplorePolicy()`
- `getSortedSemanticFrontiers()`

因为这些是“ApexNav 如何把 value map 消费成 frontier 决策”的直接实现。

### 8.3 关键判断

`hybridExplorePolicy()` 不是一直追语义热点，而是根据 frontier 语义分布统计量决定：

- 如果语义差异足够大，就 exploit semantic peaks
- 否则就退回 closest-frontier exploration

这是它比“直接 frontier 排序”更合理的地方。


## 9. `ValueMap` 为什么重要

这部分建议你精读。

文件：

- [`value_map2d.cpp`](./src/planner/plan_env/src/value_map2d.cpp)

它做的事情非常明确：

1. 把当前相机视场覆盖到的 free grids 找出来
2. 对这些格子，根据它们相对相机中心的角度，计算一个 FOV confidence
3. 用当前 ITM 分数和历史值做加权融合

核心思想不是“一个点一个分数”，而是“一个视角对一片可见区域产生语义更新”。

这个设计对你后面做 FUEL 融合有两个直接启发：

1. 语义应该先落在地图上，而不是直接落在 frontier ID 上。
2. 语义更新应当和相机视场绑定，而不是和机器人位置绑定。


## 10. `ObjectMap2D` 为什么也要读

如果你只读 `ValueMap`，会漏掉 ApexNav 的另一半语义能力。

`ObjectMap2D` 做的事情包括：

- 把单次检测到的 object point cloud 聚成对象簇
- 对同一对象的多次观测做融合
- 用 observation cloud 对“没再次看到”的对象降低置信度
- 维护最高置信度对象和 over-depth object

这意味着 ApexNav 其实有两种语义表征：

1. 连续空间语义热图：`ValueMap`
2. 离散对象级语义实体：`ObjectMap2D`

如果你未来和 `FUEL_GAZEBO` 融合，我建议两者都保留：

- `ValueMap` 影响探索方向
- `ObjectMap2D` 触发“进入目标确认/接近模式”


## 11. trajectory 模式怎么走

如果你要读 real-world / continuous trajectory 版本，只看下面这条链：

`ExplorationFSMReal -> ExplorationManager::planNextBestPoint -> ExplorationManager::planTrajectory -> KinoAstar -> GCopter -> PolyTraj -> traj_server -> /cmd_vel`

对应文件：

- [`exploration_fsm_traj.cpp`](./src/planner/exploration_manager/src/exploration_fsm_traj.cpp)
- [`exploration_manager.cpp`](./src/planner/exploration_manager/src/exploration_manager.cpp)
- [`kino_astar.cpp`](./src/planner/path_searching/src/kino_astar.cpp)
- [`optimizer.h`](./src/planner/trajectory_manager/include/trajectory_manager/optimizer.h)
- [`traj_server.cpp`](./src/planner/trajectory_manager/src/traj_server.cpp)

其中：

- `KinoAstar` 负责前端 kinodynamic 搜索
- `GCopter` 负责 MINCO/连续轨迹优化
- `traj_server` 负责把 `PolyTraj` 跟踪成 `Twist`

如果你的目标是和 `FUEL_GAZEBO` 融合，这一整段执行链基本都不是重点，因为 FUEL 自己已经有 UAV 轨迹生成和低层执行链。


## 12. Python 侧代码怎么读

### 12.1 `habitat2ros/habitat_publisher.py`

作用：

- 把 Habitat 观测发布成 ROS 话题
- 发布：
  - `/habitat/camera_depth`
  - `/habitat/camera_rgb`
  - `/habitat/odom`
  - `/habitat/sensor_pose`

这个文件主要是仿真器桥接层，不是语义策略本身。

### 12.2 `real_world_test_example/real_world_test_habitat.py`

这是 Python 语义输入主入口，建议重点读。

它负责：

- 订阅 RGB / depth / sensor pose / odom
- 调用 GroundingDINO / YOLO / SAM / BLIP2-ITM
- 发布：
  - `/detector/clouds_with_scores`
  - `/blip2/cosine_score`
  - `/detector/confidence_threshold`
  - `/detector/detect_img`

也就是说，`MapROS` 里收到的语义输入，基本都来自这个节点。

你可以把这个文件看成：

- ApexNav 的“视觉语言感知前端”

而 C++ planner 看成：

- ApexNav 的“语义搜索后端”


## 13. 如果你的目标是和 FUEL_GAZEBO 融合，优先读哪些文件

下面是我建议的最小精读集合。

### 第一优先级：必须读

- [`value_map2d.cpp`](./src/planner/plan_env/src/value_map2d.cpp)
- [`object_map2d.cpp`](./src/planner/plan_env/src/object_map2d.cpp)
- [`map_ros.cpp`](./src/planner/plan_env/src/map_ros.cpp)
- [`exploration_manager.cpp`](./src/planner/exploration_manager/src/exploration_manager.cpp)
- [`real_world_test_habitat.py`](./real_world_test_example/real_world_test_habitat.py)

理由：

- 这些文件定义了 ApexNav 的语义来源、语义表示、语义更新、语义决策。

### 第二优先级：需要知道接口，但不用深挖

- [`frontier_map2d.cpp`](./src/planner/plan_env/src/frontier_map2d.cpp)
- [`sdf_map2d.cpp`](./src/planner/plan_env/src/sdf_map2d.cpp)

理由：

- 它们决定语义最终挂载在哪种地图结构上。

### 第三优先级：可以最后看

- [`exploration_fsm.cpp`](./src/planner/exploration_manager/src/exploration_fsm.cpp)
- [`exploration_fsm_traj.cpp`](./src/planner/exploration_manager/src/exploration_fsm_traj.cpp)
- [`kino_astar.cpp`](./src/planner/path_searching/src/kino_astar.cpp)
- [`optimizer.h`](./src/planner/trajectory_manager/include/trajectory_manager/optimizer.h)
- [`traj_server.cpp`](./src/planner/trajectory_manager/src/traj_server.cpp)

理由：

- 这些主要是 ApexNav 自己的执行层，不是你和 FUEL 融合时最核心的资产。


## 14. 阅读时建议带着这几个问题

每读完一块，建议自己回答一次。

### 14.1 读完 `map_ros.cpp`

问自己：

- 哪些 ROS 话题会更新几何地图？
- 哪些 ROS 话题会更新语义地图？
- depth 和 ITM 是怎样耦合进同一张地图的？

### 14.2 读完 `value_map2d.cpp`

问自己：

- 为什么它更新的是 free grids，而不是 frontier cells？
- 这个 value map 是“语义概率图”还是“语义价值图”？
- 如果移植到 FUEL，应该放在地图层还是 frontier 层？

### 14.3 读完 `object_map2d.cpp`

问自己：

- ApexNav 如何定义“同一个对象”？
- 多次检测的 confidence 是怎么融合的？
- “没看到目标”是如何降低置信度的？

### 14.4 读完 `exploration_manager.cpp`

问自己：

- ApexNav 什么时候追 object，什么时候追 frontier？
- semantic mode 和 hybrid mode 的差别是什么？
- 它是把语义当 hard constraint 还是 soft preference？


## 15. 你后面做融合时的直接结论

基于当前代码结构，如果以后你要把 ApexNav 接到 `FUEL_GAZEBO` 上，我的建议是：

### 迁移的东西

- `ValueMap` 思路
- `ObjectMap2D` 思路
- `MapROS` 里语义输入融合逻辑
- `ExplorationManager` 的 hybrid semantic policy

### 不迁移的东西

- `traj_server`
- `KinoAstar`
- `GCopter`
- Habitat 离散动作链

因为这些执行层在 `FUEL_GAZEBO` 里已经被：

- `frontier_finder`
- `fast_exploration_manager`
- `traj_server`
- `px4ctrl`

这一整套无人机链替代了。


## 16. 一句话版本

如果只用一句话概括 ApexNav：

它不是一个“纯 frontier planner”，而是一个把 `几何 frontier`、`对象级语义实体`、`视场语义价值图` 三者融合起来做目标搜索决策的系统。

如果只用一句话概括你应该先读什么：

先读 `MapROS / ValueMap / ObjectMap2D / ExplorationManager`，最后再看执行层。
