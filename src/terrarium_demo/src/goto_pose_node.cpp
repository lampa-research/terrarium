#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <string>

class GotoPoseController : public rclcpp::Node {
public:
  GotoPoseController(const std::string &turtle_name, double target_x, double target_y, double k_angular)
      : Node("goto_pose_" + turtle_name),
        turtle_name_(turtle_name),
        target_x_(target_x),
        target_y_(target_y),
        k_angular_(k_angular) {
    // Subscribe to turtleX/pose
    std::string pose_topic = turtle_name_ + "/pose";
    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, 10,
        std::bind(&GotoPoseController::pose_callback, this, std::placeholders::_1));

    // Publisher for turtleX/cmd_vel
    std::string cmd_vel_topic = turtle_name_ + "/cmd_vel";
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic, 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GotoPoseController::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "GotoPose Controller for %s started", turtle_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Target: [%.2f, %.2f], K=%.2f", target_x_, target_y_, k_angular_);
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", pose_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", cmd_vel_topic.c_str());
  }

private:
  double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q) {
    // Calculate yaw from quaternion (assumes 2D motion, no roll/pitch)
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  double wrap_angle(double angle) {
    // Wrap angle to [-pi, pi]
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
    current_yaw_ = get_yaw_from_quaternion(current_pose_.orientation);
    pose_received_ = true;
  }

  void control_loop() {
    if (!pose_received_) {
      return;
    }

    double dx = target_x_ - current_pose_.position.x;
    double dy = target_y_ - current_pose_.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    auto cmd_vel_msg = geometry_msgs::msg::TwistStamped();
    cmd_vel_msg.header.stamp = this->now();
    cmd_vel_msg.header.frame_id = turtle_name_;

    // Stop if within tolerance
    if (distance < position_tolerance_) {
      cmd_vel_msg.twist.linear.x = 0.0;
      cmd_vel_msg.twist.angular.z = 0.0;
      cmd_vel_publisher_->publish(cmd_vel_msg);

      if (!target_reached_) {
        RCLCPP_INFO(this->get_logger(), "Target reached!");
        target_reached_ = true;
      }
      return;
    }

    target_reached_ = false;

    // Calculate angle to target
    double angle_to_target = std::atan2(dy, dx);
    double angle_error = wrap_angle(angle_to_target - current_yaw_);

    // Linear velocity: distance limited to max_linear_vel_
    double linear_vel = std::min(distance, max_linear_vel_);

    // Angular velocity: K * angle_error
    double angular_vel = k_angular_ * angle_error;

    cmd_vel_msg.twist.linear.x = linear_vel;
    cmd_vel_msg.twist.angular.z = angular_vel;

    cmd_vel_publisher_->publish(cmd_vel_msg);

    RCLCPP_DEBUG(this->get_logger(),
                 "dist=%.2f, angle_err=%.2f, lin=%.2f, ang=%.2f",
                 distance, angle_error, linear_vel, angular_vel);
  }

  std::string turtle_name_;
  double target_x_;
  double target_y_;
  double k_angular_;
  double max_linear_vel_ = 0.1;
  double position_tolerance_ = 0.05;

  bool pose_received_ = false;
  bool target_reached_ = false;
  geometry_msgs::msg::Pose current_pose_;
  double current_yaw_ = 0.0;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  if (argc < 5) {
    std::cerr << "Usage: goto_pose_node <turtle_name> <target_x> <target_y> <k_angular>" << std::endl;
    std::cerr << "Example: goto_pose_node turtle1 1.0 2.0 0.5" << std::endl;
    return 1;
  }

  std::string turtle_name = argv[1];
  double target_x = std::stod(argv[2]);
  double target_y = std::stod(argv[3]);
  double k_angular = std::stod(argv[4]);

  auto node = std::make_shared<GotoPoseController>(turtle_name, target_x, target_y, k_angular);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
