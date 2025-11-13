#include "PureNDT-3D-ROS2/purendt_3d_ros2.hpp"

#include <Eigen/Dense>
#include <rclcpp/node_options.hpp>
#include <stdexcept>

#include "transform.hpp"

namespace PureNDT3D
{

PureNDT3DNode::PureNDT3DNode(const rclcpp::NodeOptions & opts) : Node("pure_ndt_node", opts)
{
  TransformVec6D init_vec = TransformVec6D::Zero();
  current_transform_ = std::make_shared<TransformType>(init_vec);
  RCLCPP_INFO(this->get_logger(), "Starting PureNDT-3D ROS2 Wrapper Node...");

  NDTConfig config;
  map_frame_id_ = this->declare_parameter<std::string>("map_frame", "map");
  base_frame_id_ = this->declare_parameter<std::string>("base_frame", "base_link");
  config.voxel_resolution_m_ = this->declare_parameter<double>("voxel_resolution_m", 1.0);
  config.max_iterations_ = this->declare_parameter<int>("max_iterations", 10);
  config.score_threshold_ = this->declare_parameter<double>("score_threshold", 0.1);
  config.levenberg_marquardt_lambda_ =
    this->declare_parameter<double>("levenberg_marquardt_lambda", 1e-6);
  config.outlier_ratio_ = this->declare_parameter<double>("outlier_ratio", 0.05);
  config.use_second_order_derivative_ =
    this->declare_parameter<bool>("use_second_order_derivative", false);

  // register ros2 logger
  config.logger_ = setupLogger();

  // initialize NDTCore
  try {
    ndt_core_ = std::make_unique<NDTCore>(config);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize NDTCore: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  // --- TF Broadcaster ---
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // --- Publisher ---
  pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ndt_pose", 10);

  // --- Subscriber ---
  rclcpp::QoS target_qos(rclcpp::KeepLast(1));
  target_qos.transient_local();

  target_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/target_cloud", target_qos,
    std::bind(&PureNDT3DNode::targetCloudCallback, this, std::placeholders::_1));

  source_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/source_cloud", 10,
    std::bind(&PureNDT3DNode::sourceCloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "initialize completed. waiting for target cloud...");
}

LoggerCallback PureNDT3DNode::setupLogger()
{
  return [this](LogLevel level, const std::string & message) {
    switch (level) {
      case LogLevel::Debug:
        RCLCPP_DEBUG(this->get_logger(), "[PureNDT3Dlib] %s", message.c_str());
        break;
      case LogLevel::Info:
        RCLCPP_INFO(this->get_logger(), "[PureNDT3Dlib] %s", message.c_str());
        break;
      case LogLevel::Warning:
        RCLCPP_WARN(this->get_logger(), "[PureNDT3Dlib] %s", message.c_str());
        break;
      case LogLevel::Error:
        RCLCPP_ERROR(this->get_logger(), "[PureNDT3Dlib] %s", message.c_str());
        break;
    }
  };
}

std::vector<Point3D> PureNDT3DNode::convert_message(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  // PCL の型に変換
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*msg, pcl_cloud);

  std::vector<Point3D> eigen_points;
  eigen_points.reserve(pcl_cloud.points.size());

  // Eigen::Vector3d の std::vector に変換
  for (const auto & p : pcl_cloud.points) {
    eigen_points.emplace_back(p.x, p.y, p.z);
  }
  return eigen_points;
}

void PureNDT3DNode::targetCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Target points has been received (points_num: %d)",
    msg->width * msg->height);

  std::vector<Point3D> target_points = convert_message(msg);

  if (target_points.empty()) {
    RCLCPP_WARN(this->get_logger(), "the points of target map was zero");
    return;
  }

  ndt_core_->replace_target_points(target_points);
  is_target_set_ = true;
  RCLCPP_INFO(this->get_logger(), "Completed create NDT map.");
}

void PureNDT3DNode::sourceCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!is_target_set_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "source scan has been received, but target scan has not been received yet. ignoring this "
      "message...");
    return;
  }

  RCLCPP_DEBUG(
    this->get_logger(), "source scan has been received! (points_num: %d)",
    msg->width * msg->height);

  std::vector<Point3D> source_points = convert_message(msg);
  if (source_points.empty()) {
    RCLCPP_WARN(this->get_logger(), "Source scan points was empty.");
    return;
  }

  TransformType initial_guess = *current_transform_;

  *current_transform_ = ndt_core_->align(source_points, initial_guess);
  // -------------------------

  // --- publish results ---

  // Eigen::Matrix4d を geometry_msgs::msg::Pose に変換
  Transform4D result_matrix = current_transform_->get_matrix();
  Eigen::Isometry3d result_isometry(result_matrix);
  geometry_msgs::msg::Pose result_pose = tf2::toMsg(result_isometry);

  // PoseWithCovarianceStamped の作成
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = msg->header.stamp;
  pose_msg.header.frame_id = map_frame_id_;
  pose_msg.pose.pose = result_pose;
  // TODO: Covariance matrix
  pose_pub_->publish(pose_msg);

  // --- TF のブロードキャスト ---
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped.header.stamp = msg->header.stamp;
  tf_stamped.header.frame_id = map_frame_id_;
  tf_stamped.child_frame_id = base_frame_id_;
  tf_stamped.transform.translation.x = result_pose.position.x;
  tf_stamped.transform.translation.y = result_pose.position.y;
  tf_stamped.transform.translation.z = result_pose.position.z;
  tf_stamped.transform.rotation = result_pose.orientation;

  tf_broadcaster_->sendTransform(tf_stamped);
}

}  // namespace PureNDT3D

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PureNDT3D::PureNDT3DNode)
