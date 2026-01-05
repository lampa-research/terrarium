#include <rclcpp/rclcpp.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <map>
#include <string>

class OptitrackPosePublisher : public rclcpp::Node
{
public:
  OptitrackPosePublisher()
  : Node("optitrack_pose_publisher")
  {
    // Subscribe to the OptiTrack rigid bodies topic
    subscription_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
      "/rigid_bodies", 10,
      std::bind(&OptitrackPosePublisher::rigid_bodies_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "OptiTrack Pose Publisher started");
  }

private:
  void rigid_bodies_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
  {
    // Iterate through all rigid bodies in the message
    for (const auto& rigid_body : msg->rigidbodies)
    {
      std::string body_name = rigid_body.rigid_body_name;

      // Create topic name as turtleX/pose where X is the rigid body name
      std::string topic_name = "turtle" + body_name + "/pose";

      // Create publisher if it doesn't exist for this rigid body
      if (publishers_.find(body_name) == publishers_.end())
      {
        publishers_[body_name] = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          topic_name, 10);
        RCLCPP_INFO(this->get_logger(), "Created publisher for %s on topic %s",
                    body_name.c_str(), topic_name.c_str());
      }

      // Create PoseWithCovarianceStamped message
      geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;

      // Set header
      pose_cov_msg.header.stamp = this->now();
      pose_cov_msg.header.frame_id = "world";

      // Set pose
      pose_cov_msg.pose.pose = rigid_body.pose;

      // Initialize covariance matrix (6x6 = 36 elements)
      // Order: x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis
      // Setting diagonal elements to small values, off-diagonal to zero
      std::fill(pose_cov_msg.pose.covariance.begin(), pose_cov_msg.pose.covariance.end(), 0.0);
      pose_cov_msg.pose.covariance[0] = 0.001;   // x variance
      pose_cov_msg.pose.covariance[7] = 0.001;   // y variance
      pose_cov_msg.pose.covariance[14] = 0.001;  // z variance
      pose_cov_msg.pose.covariance[21] = 0.001;  // rotation about x variance
      pose_cov_msg.pose.covariance[28] = 0.001;  // rotation about y variance
      pose_cov_msg.pose.covariance[35] = 0.001;  // rotation about z variance

      // Publish the pose with covariance
      publishers_[body_name]->publish(pose_cov_msg);
    }
  }

  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr subscription_;
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> publishers_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OptitrackPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
