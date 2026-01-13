#include <rclcpp/rclcpp.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <map>
#include <string>
#include <cmath>

/**
 * OptiTrack SLAM Bridge Node
 *
 * This node bridges OptiTrack motion capture data to the TF tree format
 * required by slam_toolbox. It publishes:
 *
 * 1. TF: odom -> base_link (from OptiTrack pose)
 * 2. TF: map -> odom (identity, since OptiTrack is already in world frame)
 * 3. nav_msgs/Odometry on /odom (for compatibility with navigation stack)
 *
 * This effectively replaces wheel odometry with high-precision OptiTrack data,
 * allowing slam_toolbox to focus purely on mapping while localization comes
 * from the motion capture system.
 */
class OptitrackSlamBridge : public rclcpp::Node
{
public:
  OptitrackSlamBridge()
  : Node("optitrack_slam_bridge")
  {
    // Declare parameters
    this->declare_parameter<std::string>("rigid_body_name", "");
    this->declare_parameter<std::string>("robot_namespace", "");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<bool>("publish_map_odom_tf", true);
    this->declare_parameter<bool>("publish_odom_msg", true);
    this->declare_parameter<double>("z_offset", 0.0);  // Offset to place base_link on ground

    // Get parameters
    rigid_body_name_ = this->get_parameter("rigid_body_name").as_string();
    robot_namespace_ = this->get_parameter("robot_namespace").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    publish_map_odom_tf_ = this->get_parameter("publish_map_odom_tf").as_bool();
    publish_odom_msg_ = this->get_parameter("publish_odom_msg").as_bool();
    z_offset_ = this->get_parameter("z_offset").as_double();

    // Apply namespace prefix to frames if specified
    if (!robot_namespace_.empty()) {
      odom_frame_ = robot_namespace_ + "/" + odom_frame_;
      base_frame_ = robot_namespace_ + "/" + base_frame_;
      // map_frame typically stays global (no namespace)
    }

    // Initialize TF broadcasters
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    // Subscribe to OptiTrack rigid bodies
    subscription_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
      "/rigid_bodies", 10,
      std::bind(&OptitrackSlamBridge::rigid_bodies_callback, this, std::placeholders::_1));

    // Create odometry publisher if enabled
    if (publish_odom_msg_) {
      std::string odom_topic = robot_namespace_.empty() ? "/odom" : "/" + robot_namespace_ + "/odom";
      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    }

    // Publish static map -> odom transform (identity) if enabled
    // This tells slam_toolbox that our odom frame is already aligned with map
    if (publish_map_odom_tf_) {
      publish_static_map_odom_transform();
    }

    RCLCPP_INFO(this->get_logger(), "OptiTrack SLAM Bridge started");
    RCLCPP_INFO(this->get_logger(), "  Rigid body: %s", rigid_body_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Robot namespace: %s", robot_namespace_.empty() ? "(none)" : robot_namespace_.c_str());
    RCLCPP_INFO(this->get_logger(), "  TF: %s -> %s", odom_frame_.c_str(), base_frame_.c_str());
    if (publish_map_odom_tf_) {
      RCLCPP_INFO(this->get_logger(), "  TF: %s -> %s (static identity)", map_frame_.c_str(), odom_frame_.c_str());
    }
    if (publish_odom_msg_) {
      RCLCPP_INFO(this->get_logger(), "  Publishing odometry messages");
    }
  }

private:
  void publish_static_map_odom_transform()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = map_frame_;
    t.child_frame_id = odom_frame_;

    // Identity transform - OptiTrack world frame == map frame
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(t);
    RCLCPP_INFO(this->get_logger(), "Published static transform: %s -> %s",
                map_frame_.c_str(), odom_frame_.c_str());
  }

  void rigid_bodies_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
  {
    for (const auto& rigid_body : msg->rigidbodies) {
      // Check if this is the rigid body we're tracking
      if (!rigid_body_name_.empty() && rigid_body.rigid_body_name != rigid_body_name_) {
        continue;
      }

      // Use message timestamp for better clock sync across machines
      // This ensures TF timestamps match the source data timing
      rclcpp::Time stamp = msg->header.stamp;

      // Publish odom -> base_link transform
      geometry_msgs::msg::TransformStamped odom_to_base;
      odom_to_base.header.stamp = stamp;
      odom_to_base.header.frame_id = odom_frame_;
      odom_to_base.child_frame_id = base_frame_;

      odom_to_base.transform.translation.x = rigid_body.pose.position.x;
      odom_to_base.transform.translation.y = rigid_body.pose.position.y;
      odom_to_base.transform.translation.z = rigid_body.pose.position.z + z_offset_;
      odom_to_base.transform.rotation = rigid_body.pose.orientation;

      tf_broadcaster_->sendTransform(odom_to_base);

      // Publish odometry message if enabled
      if (publish_odom_msg_ && odom_publisher_) {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;

        // Pose
        odom_msg.pose.pose.position.x = rigid_body.pose.position.x;
        odom_msg.pose.pose.position.y = rigid_body.pose.position.y;
        odom_msg.pose.pose.position.z = rigid_body.pose.position.z + z_offset_;
        odom_msg.pose.pose.orientation = rigid_body.pose.orientation;

        // Covariance - OptiTrack is very accurate
        std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
        odom_msg.pose.covariance[0] = 0.001;   // x
        odom_msg.pose.covariance[7] = 0.001;   // y
        odom_msg.pose.covariance[14] = 0.001;  // z
        odom_msg.pose.covariance[21] = 0.001;  // roll
        odom_msg.pose.covariance[28] = 0.001;  // pitch
        odom_msg.pose.covariance[35] = 0.001;  // yaw

        // Velocity estimation (simple finite difference)
        if (last_pose_valid_) {
          double dt = (stamp - last_pose_time_).seconds();
          if (dt > 0.0 && dt < 0.5) {  // Sanity check
            odom_msg.twist.twist.linear.x = (rigid_body.pose.position.x - last_x_) / dt;
            odom_msg.twist.twist.linear.y = (rigid_body.pose.position.y - last_y_) / dt;
            odom_msg.twist.twist.linear.z = (rigid_body.pose.position.z - last_z_) / dt;

            // Angular velocity from quaternion difference (simplified - yaw only)
            double current_yaw = get_yaw_from_quaternion(rigid_body.pose.orientation);
            double yaw_diff = current_yaw - last_yaw_;
            // Wrap to [-pi, pi]
            while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
            while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
            odom_msg.twist.twist.angular.z = yaw_diff / dt;
          }
        }

        // Twist covariance
        std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);
        odom_msg.twist.covariance[0] = 0.01;
        odom_msg.twist.covariance[7] = 0.01;
        odom_msg.twist.covariance[14] = 0.01;
        odom_msg.twist.covariance[35] = 0.01;

        odom_publisher_->publish(odom_msg);

        // Store for velocity calculation
        last_x_ = rigid_body.pose.position.x;
        last_y_ = rigid_body.pose.position.y;
        last_z_ = rigid_body.pose.position.z;
        last_yaw_ = get_yaw_from_quaternion(rigid_body.pose.orientation);
        last_pose_time_ = stamp;
        last_pose_valid_ = true;
      }

      // Only process one rigid body if name is specified
      if (!rigid_body_name_.empty()) {
        break;
      }
    }
  }

  double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
  {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  // Parameters
  std::string rigid_body_name_;
  std::string robot_namespace_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string map_frame_;
  bool publish_map_odom_tf_;
  bool publish_odom_msg_;
  double z_offset_;

  // Velocity calculation state
  bool last_pose_valid_ = false;
  double last_x_ = 0.0;
  double last_y_ = 0.0;
  double last_z_ = 0.0;
  double last_yaw_ = 0.0;
  rclcpp::Time last_pose_time_;

  // ROS interfaces
  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OptitrackSlamBridge>());
  rclcpp::shutdown();
  return 0;
}
