#pragma once

#include <tf2_eigen/tf2_eigen.hpp>

#include "eigen3/Eigen/Core"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"

// PureNDT3D Library
#include "config.hpp"
#include "ndt_core.hpp"
#include "transform.hpp"

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Point3D = PureNDT3D::Point3D;
using Transform4D = PureNDT3D::Transform4D;
using TransformVec6D = PureNDT3D::TransformVec6D;

namespace PureNDT3D
{

class PureNDT3DNode : public rclcpp::Node
{
public:
  PureNDT3DNode(const rclcpp::NodeOptions & opts);

private:
  // --- ROS 2 インターフェース ---
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr target_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr source_cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // --- コールバック ---
  void targetCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void sourceCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  std::unique_ptr<NDTCore> ndt_core_;
  std::shared_ptr<TransformType> current_transform_;
  bool is_target_set_ = false;

  std::string map_frame_id_;
  std::string base_frame_id_;

  /**
   * @brief ROS の PointCloud2 メッセージを Point3D の std::vector に変換
   */
  std::vector<Point3D> convert_message(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  /**
   * @brief ROS のロガーを使用するように NDTCore の LoggerCallback
   * をセットアップ
   */
  LoggerCallback setupLogger();
};

}  // namespace PureNDT3D
