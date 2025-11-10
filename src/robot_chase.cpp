#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class RobotChase : public rclcpp::Node {
   public:
    RobotChase()
        : Node("robot_chase") {
        // Create TF buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create publisher for velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/rick/cmd_vel", 10);

        // Create timer for control loop
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&RobotChase::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Robot Chase node started");
        RCLCPP_INFO(this->get_logger(), "  kp_distance: %.2f", kp_distance_);
        RCLCPP_INFO(this->get_logger(), "  kp_yaw: %.2f", kp_yaw_);
        RCLCPP_INFO(this->get_logger(), "  max_linear_velocity: %.2f m/s", max_linear_velocity_);
        RCLCPP_INFO(this->get_logger(), "  max_angular_velocity: %.2f rad/s", max_angular_velocity_);
        RCLCPP_INFO(this->get_logger(), "  min_distance: %.2f m", min_distance_);
        RCLCPP_INFO(this->get_logger(), "Rick will now chase Morty!");
    }

   private:
    void control_loop() {
        geometry_msgs::msg::TransformStamped transform_stamped;

        try {
            // Look up the transform from rick/base_link to morty/base_link
            transform_stamped = tf_buffer_->lookupTransform(
                "rick/base_link",
                "morty/base_link",
                tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Could not transform rick/base_link to morty/base_link: %s", ex.what());
            return;
        }

        // Calculate distance and angle errors
        double error_distance = std::hypot(
            transform_stamped.transform.translation.x, transform_stamped.transform.translation.y);

        double error_yaw = std::atan2(
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.x);

        // Create velocity command
        auto cmd_vel = geometry_msgs::msg::Twist();

        // Stop if chase is completed
        if (error_distance < min_distance_) {
            // Publish velocity command
            velocity_publisher_->publish(cmd_vel);
            RCLCPP_INFO(this->get_logger(), "Chase completed! Final distance: %.2f",
                        error_distance);
            return;
        }

        // Apply proportional control for linear velocity
        double linear_velocity = kp_distance_ * error_distance;

        // Clamp to maximum linear velocity
        linear_velocity = std::min(linear_velocity, max_linear_velocity_);

        cmd_vel.linear.x = linear_velocity;

        // Calculate angular velocity
        double angular_velocity = kp_yaw_ * error_yaw;

        // Clamp to maximum angular velocity (both positive and negative)
        if (angular_velocity > max_angular_velocity_) {
            angular_velocity = max_angular_velocity_;
        } else if (angular_velocity < -max_angular_velocity_) {
            angular_velocity = -max_angular_velocity_;
        }

        cmd_vel.angular.z = angular_velocity;

        // Publish velocity command
        velocity_publisher_->publish(cmd_vel);

        // Log information periodically
        RCLCPP_DEBUG(this->get_logger(),
                     "Distance: %.2f m, Yaw: %.2f rad, Linear: %.2f m/s, Angular: %.2f rad/s",
                     error_distance, error_yaw, cmd_vel.linear.x, cmd_vel.angular.z);
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    const double kp_distance_ = 0.5;
    const double kp_yaw_ = 1.0;
    const double max_linear_velocity_ = 0.5;
    const double max_angular_velocity_ = 1.0;
    const double min_distance_ = 0.356 * 2;  // double than robot diameter
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotChase>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
