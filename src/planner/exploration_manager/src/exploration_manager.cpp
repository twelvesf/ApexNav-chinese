/**
 * @file exploration_manager.cpp
 * @brief Implementation of exploration manager for autonomous semantic navigation
 * @author Zager-Zhang
 *
 * This file implements the ExplorationManager class that handles various
 * exploration strategies including distance-based, semantic-based, hybrid,
 * and TSP-optimized frontier selection for autonomous robot exploration.
 */

#include <exploration_manager/exploration_manager.h>
#include <exploration_manager/exploration_data.h>
#include <lkh_mtsp_solver/SolveMTSP.h>
#include <plan_env/map_ros.h>
#include <path_searching/kino_astar.h>
#include <trajectory_manager/optimizer.h>

using namespace Eigen;

namespace apexnav_planner {

ExplorationManager::~ExplorationManager() = default;

void ExplorationManager::initialize(ros::NodeHandle& nh)
{
  // Initialize SDF map and get object map reference
  sdf_map_.reset(new SDFMap2D);
  sdf_map_->initMap(nh);
  object_map2d_ = sdf_map_->object_map2d_;

  // Initialize frontier map and path finder
  frontier_map2d_.reset(new FrontierMap2D(sdf_map_, nh));
  path_finder_.reset(new Astar2D);
  path_finder_->init(nh, sdf_map_);

  // Initialize exploration data and parameter containers
  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);

  // Load exploration parameters from ROS parameter server
  nh.param("exploration/policy", ep_->policy_mode_, 0);
  nh.param("exploration/sigma_threshold", ep_->sigma_threshold_, 0.030);
  nh.param("exploration/max_to_mean_threshold", ep_->max_to_mean_threshold_, 1.2);
  nh.param("exploration/max_to_mean_percentage", ep_->max_to_mean_percentage_, 0.95);
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));

  // Get map parameters for ray casting initialization
  //从 SDFMap2D 里取出地图的基础几何参数
  double resolution = sdf_map_->getResolution();//地图分辨率
  Eigen::Vector2d origin, size;
  sdf_map_->getRegion(origin, size);//origin 地图原点  size  地图整体尺寸

  // Initialize ray caster for collision checking and TSP service client
  ray_caster2d_.reset(new RayCaster2D);
  ray_caster2d_->setParams(resolution, origin);
  tsp_client_ = nh.serviceClient<lkh_mtsp_solver::SolveMTSP>("/solve_tsp", true);

  // Initialize KinoAstar and GCopter for real-world trajectory planning
  kinoastar_.reset(new KinoAstar(nh, sdf_map_));
  kinoastar_->init();
  
  Config gcopter_config(nh);
  gcopter_.reset(new Gcopter(gcopter_config, nh, sdf_map_, kinoastar_));
  
  ROS_INFO("[ExplorationManager] KinoAstar and GCopter initialized for real-world mode");
}
//决定下一步应该去追哪个目标，以及给出一条 2D 粗路径。
// 当前状态 -> 先看目标物体 -> 不行再看 frontier -> 还不行就极端兜底
int ExplorationManager::planNextBestPoint(const Vector3d& pos, const double& yaw)
{
  Vector2d pos2d = Vector2d(pos(0), pos(1));//路径起始点坐标
  ros::Time t1 = ros::Time::now();
  auto t2 = t1;

  // Clear previous planning results
  ed_->tsp_tour_.clear();
  ed_->next_best_path_.clear();
  // 定义一个数组，里面每个元素都是“一个点云的智能指针”。点云列表
  vector<pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> object_clouds;
  sdf_map_->object_map2d_->getTopConfidenceObjectCloud(object_clouds);//严格模式

  // ==================== Navigation Mode: High-Confidence Objects ====================
  if (!object_clouds.empty()) {
    ROS_WARN("[Navigation Mode] Get object_cloud num = %ld", object_clouds.size());

    // Try to find path to each detected object in order of confidence
    for (auto object_cloud : object_clouds) {
      //给定当前机器人位置和一个目标对象点云，尝试找一条能靠近该对象的 2D 路径。
      if (searchObjectPath(pos, object_cloud, ed_->next_pos_, ed_->next_best_path_))
        return SEARCH_BEST_OBJECT;
    }
  }
  //看见了，但太远，还不够确定的目标对象
  // ==================== Navigation Mode: Over-Depth Objects ====================
  if (!object_map2d_->over_depth_object_cloud_->points.empty()) {
    ROS_WARN("[Navigation Mode (Over Depth)] Get over depth object cloud");
    if (searchObjectPath(
            pos, object_map2d_->over_depth_object_cloud_, ed_->next_pos_, ed_->next_best_path_))
      return SEARCH_OVER_DEPTH_OBJECT;
  }

  // ==================== Exploration Mode: Frontier-Based Planning ====================
  // 先缓存一个“最可疑的对象候选”，以防 frontier 路径找不到时还能退回去追它。
  sdf_map_->object_map2d_->getTopConfidenceObjectCloud(object_clouds, false);
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> top_object_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (object_clouds.size() >= 1)
    top_object_cloud = object_clouds[0];

  // Apply selected exploration policy to choose next frontier
  Eigen::Vector2d next_best_pos;
  std::vector<Eigen::Vector2d> next_best_path;
  //探索策略
  chooseExplorationPolicy(pos2d, ed_->frontier_averages_, next_best_pos, next_best_path);

  // Handle case when no passable frontiers are found
  if (next_best_path.empty()) {
    ROS_WARN("Maybe no passable frontier.");

    // Try suspicious objects as backup 找不到可达frontier 去找最可疑的对象候选
    if (!top_object_cloud->points.empty() &&
        searchObjectPath(pos, top_object_cloud, ed_->next_pos_, ed_->next_best_path_))
      return SEARCH_SUSPICIOUS_OBJECT;
    else
      // Try dormant frontiers as last resort
      chooseExplorationPolicy(
          pos2d, ed_->dormant_frontier_averages_, next_best_pos, next_best_path);

    // Extreme search mode when all normal options fail
    if (next_best_path.empty()) {
      ROS_ERROR("search exterme case!!!");

      // Try extreme object search with relaxed constraints 对非严格模式下的对象进行extreme路径搜索 放宽了路径搜索约束
      for (auto object_cloud : object_clouds) {
        if (!object_cloud->points.empty() &&
            searchObjectPathExtreme(pos, object_cloud, ed_->next_pos_, ed_->next_best_path_))
          return SEARCH_EXTREME;
      }

      // Include lower confidence objects in extreme search 既放宽了对象候选集合本身，又放宽了路径搜索约束
      sdf_map_->object_map2d_->getTopConfidenceObjectCloud(object_clouds, false, true);
      for (auto object_cloud : object_clouds) {
        if (!object_cloud->points.empty() &&
            searchObjectPathExtreme(pos, object_cloud, ed_->next_pos_, ed_->next_best_path_))
          return SEARCH_EXTREME;
      }

      // Try cached over-depth objects as final option
      //放宽了目标候选的时效性
      //last_over_depth_object_cloud：跨周期缓存的object cloud
      //即使当前帧已经看不到 over-depth 目标了，只要历史缓存里还保留着，就再极限尝试一次。
      static auto last_over_depth_object_cloud = object_map2d_->over_depth_object_cloud_;
      if (!object_map2d_->over_depth_object_cloud_->points.empty())
        last_over_depth_object_cloud = object_map2d_->over_depth_object_cloud_;

      if (!last_over_depth_object_cloud->points.empty() &&
          searchObjectPathExtreme(
              pos, last_over_depth_object_cloud, ed_->next_pos_, ed_->next_best_path_)) {
        return SEARCH_EXTREME;
      }
    }

    // Final error handling when no valid targets exist
    if (next_best_path.empty()) {
      if (ed_->frontiers_.empty()) {
        ROS_ERROR("No coverable frontier!!");
        return NO_COVERABLE_FRONTIER;
      }
      else {
        ROS_ERROR("No passable frontier!!");
        return NO_PASSABLE_FRONTIER;
      }
    }
  }

  // Store successful planning results
  //ed_ 里主要存高层探索得到的目标点和 2D 粗路径
  ed_->next_pos_ = next_best_pos;
  ed_->next_best_path_ = next_best_path;

  // Performance monitoring
  double total_time = (ros::Time::now() - t2).toSec();
  ROS_ERROR_COND(total_time > 0.25, "[Plan NBV] Total time %.2lf s too long!!!", total_time);

  return EXPLORATION;
}
//根据当前配置的探索策略，决定用哪一种 frontier 选择方法
void ExplorationManager::chooseExplorationPolicy(Vector2d cur_pos, vector<Vector2d> frontiers,
    Vector2d& next_best_pos, vector<Vector2d>& next_best_path)
{
  switch (ep_->policy_mode_) {
    case ExplorationParam::DISTANCE:
      ROS_WARN("[Exploration Mode] Find Closest Frontier");
      findClosestFrontierPolicy(cur_pos, frontiers, next_best_pos, next_best_path);
      break;

    case ExplorationParam::SEMANTIC:
      ROS_WARN("[Exploration Mode] Find Highest Semantic Value Frontier");
      findHighestSemanticsFrontierPolicy(cur_pos, frontiers, next_best_pos, next_best_path);
      break;

    case ExplorationParam::HYBRID:
      ROS_WARN("[Exploration Mode] Working on Hybrid Mode");
      hybridExplorePolicy(cur_pos, frontiers, next_best_pos, next_best_path);
      break;

    case ExplorationParam::TSP_DIST:
      ROS_WARN("[Exploration Mode] Working on TSP Distance Mode");
      findTSPTourPolicy(cur_pos, frontiers, next_best_pos, next_best_path);
      break;

    default:
      ROS_WARN("[Exploration Mode] Unknown Mode");
      break;
  }
}
//先看当前所有 frontier 的语义价值分布，如果语义热点足够明显，就优先去追高语义 frontier；
//如果不明显，就退回普通几何探索，去最近 frontier
//输出下一个点坐标和路径
void ExplorationManager::hybridExplorePolicy(Vector2d cur_pos, vector<Vector2d> frontiers,
    Vector2d& next_best_pos, vector<Vector2d>& next_best_path)
{
  //语义 frontier 值标准差的阈值，判断当前各个frontier语义值分布是否拉开差距
  double std_dev_threshold = ep_->sigma_threshold_;
  //最大语义值/平均语义值的阈值，衡量最强热点是否足够突出
  double max_to_mean_threshold = ep_->max_to_mean_threshold_;
  vector<SemanticFrontier> sem_frontiers;
  //给每个 frontier 算语义值，并排序
  getSortedSemanticFrontiers(cur_pos, frontiers, sem_frontiers);
  if (sem_frontiers.empty())
    return;

  double std_dev, max_to_mean, mean;
  //统计frontier的标准差，最大值/平均值，平均值
  calcSemanticFrontierInfo(sem_frontiers, std_dev, max_to_mean, mean);

  // Decide between exploitation and exploration based on semantic statistics
  if (std_dev > std_dev_threshold && max_to_mean > max_to_mean_threshold) {
    //语义足够突出
    ROS_WARN("Exploit the semantic value (TSP)!!");
    vector<Vector2d> high_sem_frontiers;

    // Select high-value frontiers for TSP optimization  选出高语义frontier 动态调节高语义筛选比例
    for (auto sem_frontier : sem_frontiers) {
      double auto_max_to_mean_threshold =
          max(max_to_mean_threshold, ep_->max_to_mean_percentage_ * max_to_mean);
      if (sem_frontier.semantic_value / mean < auto_max_to_mean_threshold)
        break;
      high_sem_frontiers.push_back(sem_frontier.position);
    }
    //先对一组 frontier 计算一个 TSP 访问顺序，再从这个顺序里选出当前应该先去的下一个 frontier。
    findTSPTourPolicy(cur_pos, high_sem_frontiers, next_best_pos, next_best_path);
  }
  else {
    ROS_WARN("Explore the environment (Closest)!!");
    findClosestFrontierPolicy(cur_pos, frontiers, next_best_pos, next_best_path);
  }
}

void ExplorationManager::findHighestSemanticsFrontierPolicy(Vector2d cur_pos,
    vector<Vector2d> frontiers, Vector2d& next_best_pos, vector<Vector2d>& next_best_path)
{
  next_best_path.clear();

  // Container for frontier-value pairs for sorting
  vector<pair<Vector2d, double>> frontier_values;

  // Compute semantic value for each frontier
  for (auto frontier : frontiers) {
    Vector2i idx;
    sdf_map_->posToIndex(frontier, idx);
    auto nbrs = allNeighbors(idx, 2);  // 5x5 neighborhood

    // Find maximum semantic value in local neighborhood
    double value = sdf_map_->value_map_->getValue(idx);
    for (auto nbr : nbrs) value = max(value, sdf_map_->value_map_->getValue(nbr));

    frontier_values.emplace_back(frontier, value);
  }

  // Sort by semantic value (descending), then by distance (ascending)
  auto compareFrontiers = [&cur_pos](
                              const pair<Vector2d, double>& a, const pair<Vector2d, double>& b) {
    if (fabs(a.second - b.second) > 1e-5) {
      return a.second > b.second;  // Higher semantic value first
    }
    else {
      double dist_a = (a.first - cur_pos).norm();
      double dist_b = (b.first - cur_pos).norm();
      return dist_a < dist_b;  // Closer distance first for tie-breaking
    }
  };

  std::sort(frontier_values.begin(), frontier_values.end(), compareFrontiers);

  // Update frontier list with sorted order
  frontiers.clear();
  for (const auto& fv : frontier_values) {
    frontiers.push_back(fv.first);
  }

  // Select first reachable frontier from sorted list
  for (int i = 0; i < (int)frontiers.size(); i++) {
    std::vector<Eigen::Vector2d> tmp_path;
    Eigen::Vector2d tmp_pos;
    if (!searchFrontierPath(cur_pos, frontiers[i], tmp_pos, tmp_path))
      continue;
    next_best_pos = tmp_pos;
    next_best_path = tmp_path;
    break;
  }
}
//在所有 frontier 里，找一个实际路径最短、最容易到达的 frontier。
//输入起始位置，frontier中心点，输出下一个位置和路径
void ExplorationManager::findClosestFrontierPolicy(Vector2d cur_pos, vector<Vector2d> frontiers,
    Vector2d& next_best_pos, vector<Vector2d>& next_best_path)
{
  next_best_path.clear();

  // Sort frontiers by Euclidean distance for efficient processing
  //欧式距离排序做剪枝，最后根据真实a*路径长度选优
  std::sort(frontiers.begin(), frontiers.end(), [&cur_pos](const Vector2d& a, const Vector2d& b) {
    return (a - cur_pos).norm() < (b - cur_pos).norm();
  });
  //初始化当前最短路径长度为无穷大
  double min_len = std::numeric_limits<double>::max();

  // Find the frontier with shortest actual path length
  for (int i = 0; i < (int)frontiers.size(); i++) {
    // Skip if Euclidean distance already exceeds best path length
    if ((frontiers[i] - cur_pos).norm() >= min_len)
      continue;

    std::vector<Eigen::Vector2d> tmp_path;
    Eigen::Vector2d tmp_pos;

    // Attempt path planning to this frontier  不可达就跳过，可达就判断是否最短，更新数据
    if (!searchFrontierPath(cur_pos, frontiers[i], tmp_pos, tmp_path))
      continue;

    // Update best solution if this path is shorter
    double len = Astar2D::pathLength(tmp_path);
    if (len < min_len) {
      min_len = len;
      next_best_pos = tmp_pos;
      next_best_path = tmp_path;
    }
  }
}

void ExplorationManager::findTSPTourPolicy(Vector2d cur_pos, vector<Vector2d> frontiers,
    Vector2d& next_best_pos, vector<Vector2d>& next_best_path)
{
  next_best_path.clear();
  vector<Vector2d> filter_frontiers;  //滤除当前不可达frontier
  for (auto frontier : frontiers) {
    Vector2d tmp_pos;
    vector<Vector2d> tmp_path;
    if (searchFrontierPath(cur_pos, frontier, tmp_pos, tmp_path))
      filter_frontiers.push_back(frontier);
  }

  vector<int> indices;
  //对可达frontiers计算atsp顺序，返回一组索引indices，代表frontiers的访问顺序
  computeATSPTour(cur_pos, filter_frontiers, indices);
  ed_->tsp_tour_.push_back(cur_pos);
  //atsp路线存进ed_，主要是为了可视化，本轮规划得到的 frontier 巡回顺序轨迹。
  for (auto idx : indices) ed_->tsp_tour_.push_back(filter_frontiers[idx]);

  //选择地一个可达的frontier
  if (!indices.empty()) {
    for (auto idx : indices) {
      Vector2d next_bext_frontier = filter_frontiers[idx];
      if (searchFrontierPath(cur_pos, next_bext_frontier, next_best_pos, next_best_path))
        break;
    }
  }
}

double ExplorationManager::computePathCost(const Vector2d& pos1, const Vector2d& pos2)
{
  path_finder_->reset();
  if (path_finder_->astarSearch(pos1, pos2, 0.25, 0.002) == Astar2D::REACH_END)
    return Astar2D::pathLength(path_finder_->getPath());
  return 10000.0;
}

void ExplorationManager::computeATSPCostMatrix(
    const Vector2d& cur_pos, const vector<Vector2d>& frontiers, Eigen::MatrixXd& mat)
{
  int dimen = frontiers.size() + 1;
  mat.resize(dimen, dimen);

  // Agent to frontiers
  for (int i = 1; i < dimen; i++) {
    mat(0, i) = computePathCost(cur_pos, frontiers[i - 1]);
    mat(i, 0) = 0;
  }

  // Costs between frontiers
  for (int i = 1; i < dimen; ++i) {
    for (int j = i + 1; j < dimen; ++j) {
      double cost = computePathCost(frontiers[i - 1], frontiers[j - 1]);
      mat(i, j) = cost;
      mat(j, i) = cost;
    }
  }

  // Diag
  for (int i = 0; i < dimen; ++i) {
    mat(i, i) = 100000.0;
  }
}

void ExplorationManager::computeATSPTour(
    const Vector2d& cur_pos, const vector<Vector2d>& frontiers, vector<int>& indices)
{
  indices.clear();
  if (frontiers.empty()) {
    ROS_ERROR("No frontier to compute tsp!");
    return;
  }
  else if (frontiers.size() == 1) {
    indices.push_back(0);
    return;
  }
  /* change ATSP to lhk3 */
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  computeATSPCostMatrix(cur_pos, frontiers, cost_mat);
  const int dimension = cost_mat.rows();

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Initialize ATSP par file
  // Create problem file
  ofstream file(ep_->tsp_dir_ + "/atsp_tour.atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * cost_mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  const int drone_num = 1;
  file.open(ep_->tsp_dir_ + "/atsp_tour.par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->tsp_dir_ + "/atsp_tour.atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->tsp_dir_ + "/atsp_tour.tour\n";
  file.close();

  auto par_dir = ep_->tsp_dir_ + "/atsp_tour.atsp";

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 1;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return;
  }

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/atsp_tour.tour");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }

  // Read path for ATSP formulation
  while (getline(res_file, res)) {
    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == 1)  // Ignore the current state
      continue;
    if (id == -1)
      break;
    indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
  }

  res_file.close();

  // for (auto idx : indices) ROS_WARN("ATSP idx = %d", idx);

  double tsp_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("[ATSP Tour] Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
}
//在一个对象点云里，找出离机器人当前位置最近的那个点。
Vector2d ExplorationManager::findNearestObjectPoint(
    const Vector3d& start, const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& object_cloud)
{
  // 建一个 KD-Tree
//   KD-Tree 是一种用来做最近邻搜索的数据结构。
// 这里就是为了高效找“最近点”。
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(object_cloud);
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);
  //把机器人当前位置转成 PCL 点格式。
  pcl::PointXYZ cur_pt;
  cur_pt.x = start(0);
  cur_pt.y = start(1);
  cur_pt.z = start(2);
  //从对象点云里找离当前机器人最近的 1 个点。
  if (kdtree.nearestKSearch(cur_pt, 1, pointIdxNKNSearch, pointNKNSquaredDistance) <= 0) {
    ROS_ERROR("[Bug] No nearest object point found.");
    return Vector2d(-1000.0, -1000.0);  // Error indicator
  }

  int nearest_idx = pointIdxNKNSearch[0];
  auto nearest_point = object_cloud->points[nearest_idx];
  return Vector2d(nearest_point.x, nearest_point.y);
}

bool ExplorationManager::trySearchObjectPathWithDistance(const Vector2d& start2d,
    const Vector2d& object_pose, double distance, double max_search_time,
    Eigen::Vector2d& refined_pos, std::vector<Eigen::Vector2d>& refined_path,
    const std::string& debug_msg)
{
  path_finder_->reset();
  //证明可以靠近，但是这时路径终点仍然为onject_pose
  if (path_finder_->astarSearch(start2d, object_pose, distance, max_search_time) ==
      Astar2D::REACH_END) {
    std::vector<Eigen::Vector2d> path = path_finder_->getPath();
    Vector2d tmp_pos(-1000.0, -1000.0);

    // Find valid position along the path (from end to start)
    //寻找实际可执行的安全点
    for (int i = path.size() - 1; i >= 0; i--) {
      //path[i]：路径上的第 i 个二维路径点/waypoint
      if (sdf_map_->getOccupancy(path[i]) != SDFMap2D::OCCUPIED &&
          sdf_map_->getOccupancy(path[i]) != SDFMap2D::UNKNOWN &&
          sdf_map_->getInflateOccupancy(path[i]) != 1) {
        tmp_pos = path[i];
        break;
      }
    }

    // Search path to the valid position
    path_finder_->reset();
    if (path_finder_->astarSearch(start2d, tmp_pos, 0.2, max_search_time) == Astar2D::REACH_END) {
      refined_path = path_finder_->getPath();
      refined_pos = tmp_pos;
      if (!debug_msg.empty()) {
        ROS_WARN("%s", debug_msg.c_str());
      }
      return true;
    }
  }
  return false;
}
//给定当前机器人位置和一个目标对象点云，尝试找一条能靠近该对象的 2D 路径。
bool ExplorationManager::searchObjectPath(const Vector3d& start,
    const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& object_cloud,
    Eigen::Vector2d& refined_pos, std::vector<Eigen::Vector2d>& refined_path)
{
  const double max_search_time = 0.2;  // Maximum planning time per attempt
  Vector2d start2d = Vector2d(start(0), start(1));//把当前3d起点取成2d

  // Find nearest accessible point in object cloud：从目标对象点云里找一个“最近且可接近”的对象位置
  Vector2d object_pose = findNearestObjectPoint(start, object_cloud);
  if (object_pose.x() < -999.0)
    return false;  // Error indicator from findNearestObjectPoint

  // Try different safety distances in order of preference
  const std::vector<double> distances = { 0.5, 0.70, 0.85 };
  const std::vector<std::string> debug_messages = { "I'm going to the object! dist = 0.5m!",
    "I'm going to the object! dist = 0.70m!", "I'm going to the object! dist = 0.85m!" };
  // 找去目标附近一个合适观察/接近点的路径。
  // Attempt path planning with each safety distance
  for (size_t i = 0; i < distances.size(); ++i) {
    if (trySearchObjectPathWithDistance(start2d, object_pose, distances[i], max_search_time,
            refined_pos, refined_path, debug_messages[i])) {
      return true;
    }
  }

  ROS_ERROR("Failed to find object path.");
  return false;
}
//给每个 frontier 计算“语义价值 + 可达路径”，然后把可达的 frontier 按优先级排好序。
void ExplorationManager::getSortedSemanticFrontiers(const Vector2d& cur_pos,
    const vector<Vector2d>& frontiers, vector<SemanticFrontier>& sem_frontiers)
{
  // Filter and sort frontiers based on semantic values and reachability
  sem_frontiers.clear();
  //给每个 frontier 建一个 SemanticFrontier 结构（position,semantic_value,path_length,path）
  for (auto& frontier : frontiers) {
    SemanticFrontier sem_frontier;
    sem_frontier.position = frontier;

    // Compute semantic value from local neighborhood
    //frontier算语义值（局部邻域最大语义值）
    //idx:栅格地图里的二维索引
    Vector2i idx;
    sdf_map_->posToIndex(frontier, idx);
    auto nbrs = allNeighbors(idx, 2);  // 5x5 grid neighborhood 找以 idx 为中心，半径 2 格的邻居集合。
    double value = sdf_map_->value_map_->getValue(idx);

    // Find maximum semantic value in neighborhood (ignoring occupied cells)
    for (auto& nbr : nbrs) {
      if (sdf_map_->getInflateOccupancy(nbr) == 1 ||
          sdf_map_->getOccupancy(nbr) == SDFMap2D::OCCUPIED)
        continue;
      value = std::max(value, sdf_map_->value_map_->getValue(nbr));
    }
    sem_frontier.semantic_value = value;

    // Validate reachability and compute path cost
    Vector2d tmp_pos;
    vector<Vector2d> tmp_path;
    if (!searchFrontierPath(cur_pos, frontier, tmp_pos, tmp_path)) {
      // Assign high cost penalty for unreachable frontiers
      sem_frontier.path_length = 1000000;
      sem_frontier.path.clear();
    }
    else {
      sem_frontier.path_length = Astar2D::pathLength(tmp_path);
      sem_frontier.path = tmp_path;
    }

    // Only include frontiers with valid paths
    if (!sem_frontier.path.empty())
      sem_frontiers.push_back(sem_frontier);
  }

  // Sort by semantic value (desc) then by path length (asc),
  //semantic_value 从高到低,path_length 从短到长 头文件中定义
  std::sort(sem_frontiers.begin(), sem_frontiers.end());
}

void ExplorationManager::calcSemanticFrontierInfo(const vector<SemanticFrontier>& sem_frontiers,
    double& std_dev, double& max_to_mean, double& mean, bool if_print)
{
  // Handle empty frontier list
  if (sem_frontiers.empty()) {
    std::cout << "No semantic frontiers available." << std::endl;
    max_to_mean = 1.0;  // Neutral ratio
    std_dev = 0.0;      // No variation
    return;
  }

  // Compute mean and maximum semantic values
  double sum = 0.0;
  double max_value = 0.0;
  for (const auto& frontier : sem_frontiers) {
    sum += frontier.semantic_value;
    max_value = max(max_value, frontier.semantic_value);
  }
  mean = sum / sem_frontiers.size();

  // Compute standard deviation
  double variance_sum = 0.0;
  for (const auto& frontier : sem_frontiers)
    variance_sum += (frontier.semantic_value - mean) * (frontier.semantic_value - mean);

  max_to_mean = max_value / mean;
  std_dev = std::sqrt(variance_sum / sem_frontiers.size());

  // Print summary statistics
  std::cout << "Mean Value: " << std::fixed << std::setprecision(3) << mean;
  std::cout << " , Standard Deviation: " << std::fixed << std::setprecision(3) << std_dev;
  std::cout << " , Max-to-Mean: " << std::fixed << std::setprecision(3) << max_to_mean << std::endl;

  // Print detailed frontier values if requested
  if (if_print) {
    for (const auto& sem_frontier : sem_frontiers)
      std::cout << "Value: " << std::fixed << std::setprecision(3) << sem_frontier.semantic_value
                << std::endl;
  }
}
//把上层给出的起点状态和目标状态，变成一条真正连续可执行的局部轨迹。
bool ExplorationManager::planTrajectory(
    const Eigen::VectorXd& start, const Eigen::VectorXd& end, const Vector3d& ctrl)
{
  if (!gcopter_ || !kinoastar_) {
    ROS_WARN_THROTTLE(1.0, "[ExplorationManager] GCopter or KinoAstar not initialized for real-world mode");
    return false;
  }
  
  Eigen::VectorXd goal_state, current_state;
  Vector3d control = ctrl;
  goal_state = end;
  current_state = start;

  // Kinodynamic A* search 先找一条满足动力学约束的粗可行轨迹/初始轨迹。
  kinoastar_->reset();
  kinoastar_->search(goal_state, current_state, control);
  //把 KinoAstar 已经搜到的结果整理成后端能用的轨迹表示
  kinoastar_->getKinoNode();
  
  if (kinoastar_->has_path_) {
    kinoastar_->kinoastarFlatPathPub(kinoastar_->flat_trajs_);
    //轨迹优化：更平滑、更连续、满足约束的多项式轨迹
    gcopter_->minco_plan();
    std::vector<Trajectory<7, 3>> final_trajes = gcopter_->final_trajes;
    //把已经算好的最终 MINCO/GCopter 轨迹发布成一条可在 RViz 里显示的路径
    gcopter_->mincoPathPub(gcopter_->final_trajes, gcopter_->final_singuls);
    return true;
  }
  
  return false;
}

}  // namespace apexnav_planner
