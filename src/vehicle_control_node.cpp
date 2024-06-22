#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp> // Include the header for Bool message

using namespace std::chrono_literals;

class VehicleControl : public rclcpp::Node
{
public:
  VehicleControl()
  : Node("vehicle_control_node"),
    target_speed_(0.0),
    target_distance_(0.0),
    distance_traveled_(0.0),
    acceleration(0.0),
    distance_traveled_km_h_(0.0),
    initial_velocity_(0.0),
    start_time_(now()),
    emergency_stop_triggered_(false)
  {
    this->declare_parameter("target_speed", 40.0);
    this->declare_parameter("target_distance", 20.0);

    this->get_parameter("target_speed", target_speed_);
    this->get_parameter("target_distance", target_distance_);

    RCLCPP_INFO(this->get_logger(), "Target speed: %.2f km/h", target_speed_);
    RCLCPP_INFO(this->get_logger(), "Target distance: %.2f m", target_distance_);

    // Convert from km/h to m/s
    target_speed_ = target_speed_ / 3.6;

    accel_pub_ = this->create_publisher<std_msgs::msg::Float64>("acceleration", 10);

    // subscriber for emergency stop (not implemented yet)
    emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "emergency_stop", 10, std::bind(&VehicleControl::emergency_stop_callback, this, std::placeholders::_1));


    timer_ = this->create_wall_timer(
      500ms, std::bind(&VehicleControl::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto current_time = now();
    auto time_diff = (current_time - start_time_).seconds();

    // Calculate acceleration
    acceleration = (target_speed_ - initial_velocity_) / time_diff;

    if (!emergency_stop_triggered_ && distance_traveled_ < target_distance_) {

      // Update distance traveled
      distance_traveled_ = initial_velocity_ * time_diff + 0.5 * acceleration * time_diff * time_diff;

      // Publish acceleration
      std_msgs::msg::Float64 accel_msg;
      accel_msg.data = acceleration;
      accel_pub_->publish(accel_msg);

      distance_traveled_km_h_ = distance_traveled_ * 3.6;

      RCLCPP_INFO(this->get_logger(), "Time: %.2f seconds", time_diff);
      RCLCPP_INFO(this->get_logger(), "Acceleration: %.2f m/s^2", acceleration);
      RCLCPP_INFO(this->get_logger(), "Distance traveled: %.2f km/h\n", distance_traveled_km_h_);
    } else {

      if(emergency_stop_triggered_){
        RCLCPP_INFO(this->get_logger(), "Emergency stop triggered. Stopping the vehicle.");

        // Calculate remaining distance to stop within 8.5 meters
        double remaining_distance = 8.5 - distance_traveled_;
        double stopping_time = sqrt(2 * remaining_distance / acceleration);

        target_speed_ = acceleration * stopping_time;

        start_time_ = now();

        distance_traveled_ = 0.0;

        emergency_stop_triggered_ = false;
      }


      RCLCPP_INFO(this->get_logger(), "Target distance reached.");
      timer_->cancel();
      return;
    }
  }

  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      emergency_stop_triggered_ = true;
      RCLCPP_INFO(this->get_logger(), "Emergency stop signal received.");
    }
  }

  rclcpp::Time now() {
    return this->get_clock()->now();
  }

  double target_speed_;
  double target_distance_;
  double distance_traveled_;
  double acceleration;
  double distance_traveled_km_h_;
  double initial_velocity_;
  rclcpp::Time start_time_;
  bool emergency_stop_triggered_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr accel_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleControl>());
  rclcpp::shutdown();
  return 0;
}
