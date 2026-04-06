/**
 * @file value_map2d.cpp
 * @brief Implementation of semantic value mapping system with confidence-weighted ITM score fusion
 *
 * This file implements the ValueMap class which provides semantic value mapping capabilities
 * for autonomous navigation systems. The implementation focuses on confidence-weighted fusion
 * of ITM (Image-Text Matching) scores using field-of-view based confidence modeling.
 *
 * Reference paper "VLFM: Vision-Language Frontier Maps for Zero-Shot Semantic Navigation"
 *
 * @author Zager-Zhang
 */

#include <plan_env/value_map2d.h>

namespace apexnav_planner {
  //初始化语义价值地图的底层存储。
ValueMap::ValueMap(SDFMap2D* sdf_map, ros::NodeHandle& nh)
{
  this->sdf_map_ = sdf_map;
  int voxel_num = sdf_map_->getVoxelNum();
  //分配buffer
  value_buffer_ = vector<double>(voxel_num, 0.0); //每个栅格当前的语义价值
  confidence_buffer_ = vector<double>(voxel_num, 0.0);  //每个栅格当前这个语义价值的累计置信度/观测可信度
}

//把当前这一帧的 ITM 语义分数，融合到当前视野内所有 free grid 上，更新整张 2D 语义价值图
void ValueMap::updateValueMap(const Vector2d& sensor_pos, const double& sensor_yaw,
    const vector<Vector2i>& free_grids, const double& itm_score)
{
  //遍历当前帧所有free grids
  for (const auto& grid : free_grids) {
    Vector2d pos;
    //把格子索引转成位置和地址
    sdf_map_->indexToPos(grid, pos);
    int adr = sdf_map_->toAddress(grid);  //用来访问一维buffer

    // Calculate FOV-based confidence for current observation
    //计算当前观测对这个格子的可信度:同一帧里不同位置的观测权重不一样
    double now_confidence = getFovConfidence(sensor_pos, sensor_yaw, pos);
    //当前观测值 这一帧图像的整体语义分数,被赋值给当前看到的每个free grid  同一帧差别体现在可信度
    double now_value = itm_score;

    // Retrieve existing confidence and value 历史值
    double last_confidence = confidence_buffer_[adr];
    double last_value = value_buffer_[adr];

    // Apply confidence-weighted fusion with quadratic confidence combination  平均加权融合
    confidence_buffer_[adr] =
        (now_confidence * now_confidence + last_confidence * last_confidence) /
        (now_confidence + last_confidence);
    value_buffer_[adr] = (now_confidence * now_value + last_confidence * last_value) /
                         (now_confidence + last_confidence);
  }
}
//基于相机视场角的方向性权重函数
// 正前方：confidence 最大
// 越靠近边缘：confidence 越小
// 中心区域权重大，边缘区域权重小
double ValueMap::getFovConfidence(
    const Vector2d& sensor_pos, const double& sensor_yaw, const Vector2d& pt_pos)
{
  // Calculate relative position vector from sensor to target point
  Vector2d rel_pos = pt_pos - sensor_pos;
  double angle_to_point = atan2(rel_pos(1), rel_pos(0));

  // Normalize angles to [-π, π] range for consistent angular arithmetic
  double normalized_sensor_yaw = normalizeAngle(sensor_yaw);
  double normalized_angle_to_point = normalizeAngle(angle_to_point);
  double relative_angle = normalizeAngle(normalized_angle_to_point - normalized_sensor_yaw);

  // Apply cosine-squared FOV confidence model
  // FOV angle: 79° total field of view (typical RGB camera)
  double fov_angle = 79.0 * M_PI / 180.0;
  double value = std::cos(relative_angle / (fov_angle / 2) * (M_PI / 2));
  return value * value;  // Square for stronger center weighting
}

}  // namespace apexnav_planner
