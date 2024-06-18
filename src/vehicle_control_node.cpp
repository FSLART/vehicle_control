#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class VehicleControl : public rclcpp::Node
{
public:
  VehicleControl()
  : Node("vehicle_control_node"),
    target_speed_(0.0),
    target_distance_(0.0),
    distance_traveled_(0.0),
    initial_velocity_(0.0),
    initial_position_set_(false),
    start_time_(now())
  {
    // Declarar e obter parâmetros
    this->declare_parameter("target_speed", 40.0);
    this->declare_parameter("target_distance", 20.0);

    this->get_parameter("target_speed", target_speed_);
    this->get_parameter("target_distance", target_distance_);

    RCLCPP_INFO(this->get_logger(), "Target speed: %.2f km/h", target_speed_);
    RCLCPP_INFO(this->get_logger(), "Target distance: %.2f m", target_distance_);

    // Converter de km/h para m/s
    target_speed_ = target_speed_ / 3.6;


    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);


    timer_ = this->create_wall_timer(
      500ms, std::bind(&VehicleControl::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto current_time = now();
    auto time_diff = (current_time - start_time_).seconds();

    if (distance_traveled_ < target_distance_) {
      geometry_msgs::msg::Twist cmd_msg;
      cmd_msg.linear.x = target_speed_;
      cmd_pub_->publish(cmd_msg);

      // Calcular aceleração
      double acceleration = (target_speed_ - initial_velocity_) / time_diff;

      // Atualizar distância percorrida
      distance_traveled_ = initial_velocity_ * time_diff + 0.5 * acceleration * time_diff * time_diff;

      RCLCPP_INFO(this->get_logger(), "Time: %.2f seconds", time_diff);
      RCLCPP_INFO(this->get_logger(), "Current speed: %.2f m/s", cmd_msg.linear.x);
      RCLCPP_INFO(this->get_logger(), "Acceleration: %.2f m/s^2", acceleration);
      RCLCPP_INFO(this->get_logger(), "Distance traveled: %.2f m\n", distance_traveled_);
    } else {
      geometry_msgs::msg::Twist cmd_msg;
      cmd_msg.linear.x = 0.0; // Parar o veículo
      cmd_pub_->publish(cmd_msg);
      RCLCPP_INFO(this->get_logger(), "Target distance reached. Stopping the vehicle.");
      timer_->cancel(); // Parar o temporizador, pois atingimos a distância alvo
    }
  }

  rclcpp::Time now() {
    return this->get_clock()->now();
  }

  double target_speed_;
  double target_distance_;
  double distance_traveled_;
  double initial_velocity_;
  bool initial_position_set_;
  rclcpp::Time start_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
