#include <rclcpp/rclcpp.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <geometry_msgs/msg/pose.hpp>
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

      // Check if body_name is a number or starts with a number
      // Create topic name as turtleX/pose where X is the rigid body name
      std::string topic_name = "turtle" + body_name + "/pose";

      // Create publisher if it doesn't exist for this rigid body
      if (publishers_.find(body_name) == publishers_.end())
      {
        publishers_[body_name] = this->create_publisher<geometry_msgs::msg::Pose>(
          topic_name, 10);
        RCLCPP_INFO(this->get_logger(), "Created publisher for %s on topic %s",
                    body_name.c_str(), topic_name.c_str());
      }

      // Publish the pose
      publishers_[body_name]->publish(rigid_body.pose);
    }
  }

  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr subscription_;
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr> publishers_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OptitrackPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
