#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

class TurtleController : public rclcpp::Node {
public:
  TurtleController(const std::string &turtle_name)
      : Node("turtle_controller_" + turtle_name), turtle_name_(turtle_name) {
    // Subscribe to turtleX/pose
    std::string pose_topic = turtle_name_ + "/pose";
    pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        pose_topic, 10,
        std::bind(&TurtleController::pose_callback, this,
                  std::placeholders::_1));

    // Publisher for turtleX/cmd_vel
    std::string cmd_vel_topic = turtle_name_ + "/cmd_vel";
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic, 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TurtleController::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Turtle Controller for %s started",
                turtle_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", pose_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", cmd_vel_topic.c_str());
  }

private:
  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    // Store the latest pose
    current_pose_ = *msg;
    pose_received_ = true;

    // Example: Log the position
    RCLCPP_DEBUG(this->get_logger(),
                 "Received pose - Position: [%.2f, %.2f, %.2f]",
                 msg->position.x, msg->position.y, msg->position.z);

    // TODO: Add your logic here based on the received pose
    // For example, calculate desired velocity based on current position
  }

  void control_loop() {
    if (!pose_received_) {
      return; // Wait until we receive the first pose
    }

    // TODO: Implement your control algorithm here
    // This is called every 100ms

    // Example: Simple stationary command (no movement)
    auto cmd_vel_msg = geometry_msgs::msg::TwistStamped();
    cmd_vel_msg.header.stamp = this->now();
    cmd_vel_msg.header.frame_id = turtle_name_;
    cmd_vel_msg.twist.linear.x = 0.0;  // Forward/backward velocity
    cmd_vel_msg.twist.linear.y = 0.0;  // Left/right velocity (if supported)
    cmd_vel_msg.twist.linear.z = 0.0;  // Up/down velocity (if supported)
    cmd_vel_msg.twist.angular.x = 0.0; // Roll rate
    cmd_vel_msg.twist.angular.y = 0.0; // Pitch rate
    cmd_vel_msg.twist.angular.z = 0.0; // Yaw rate (rotation)

    // Example control logic:
    // Move forward at 0.5 m/s if x position is less than 1.0
    if (current_pose_.position.x < 1.0) {
      cmd_vel_msg.twist.linear.x = 0.5;
    } else {
      cmd_vel_msg.twist.linear.x = 0.0;
    }

    // Publish the velocity command
    cmd_vel_publisher_->publish(cmd_vel_msg);
  }

  std::string turtle_name_;
  bool pose_received_ = false;
  geometry_msgs::msg::Pose current_pose_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Get turtle name from command line argument (default: "turtle0")
  std::string turtle_name = "turtle0";
  if (argc > 1) {
    turtle_name = argv[1];
  }

  auto node = std::make_shared<TurtleController>(turtle_name);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
