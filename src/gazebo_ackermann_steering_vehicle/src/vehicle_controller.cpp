#include "vehicle_controller.hpp"
#include <geometry_msgs/msg/twist.hpp> // Añadida para Twist

VehicleController::VehicleController(const double timer_period, const double timeout_duration) :
  Node{"vehicle_controller"},
  timeout_duration_{timeout_duration},
  last_velocity_time_{get_clock()->now()},
  last_steering_time_{get_clock()->now()},
  body_width_{0.0},
  body_length_{0.0},
  wheel_radius_{0.0},
  wheel_width_{0.0},
  max_steering_angle_{0.0},
  max_velocity_{0.0},
  wheel_base_{0.0},
  track_width_{0.0},
  steering_angle_{0.0},
  velocity_{0.0},
  wheel_angular_velocity_{0.0, 0.0},
  wheel_steering_angle_{0.0, 0.0}
{
  declare_parameter<double>("body_width", 0.3);
  declare_parameter<double>("body_length", 0.5);
  declare_parameter<double>("wheel_radius", 0.05);
  declare_parameter<double>("wheel_width", 0.04);
  declare_parameter<double>("max_steering_angle", 0.5);
  declare_parameter<double>("max_velocity", 2.0);

  get_parameter("body_width", body_width_);
  get_parameter("body_length", body_length_);
  get_parameter("wheel_radius", wheel_radius_);
  get_parameter("wheel_width", wheel_width_);
  get_parameter("max_steering_angle", max_steering_angle_);
  get_parameter("max_velocity", max_velocity_);

  track_width_ = body_width_ + wheel_width_;
  wheel_base_ = body_length_ - (2 * wheel_radius_);

  // Único suscriptor para CMD_VEL
  cmd_vel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&VehicleController::cmd_vel_callback, this, std::placeholders::_1));

  // Publishers (se mantienen igual)
  position_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_position_controller/commands", 10);

  velocity_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_velocity_controller/commands", 10);

  timer_ = create_wall_timer(std::chrono::duration<double>(timer_period),
                             std::bind(&VehicleController::timer_callback, this));
}

// Lógica de Ackermann y Diferencial (Se mantienen igual, son correctas)
std::pair<double, double> VehicleController::ackermann_steering_angle()
{
  double left_wheel_angle{0.0}, right_wheel_angle{0.0};
  if (abs(steering_angle_) > 1e-3) {
    const double sin_angle = sin(abs(steering_angle_));
    const double cos_angle = cos(abs(steering_angle_));
    if (steering_angle_ > 0.0) {
      left_wheel_angle = atan((2 * wheel_base_ * sin_angle) / (2 * wheel_base_ * cos_angle - track_width_ * sin_angle));
      right_wheel_angle = atan((2 * wheel_base_ * sin_angle) / (2 * wheel_base_ * cos_angle + track_width_ * sin_angle));
    } else {
      left_wheel_angle = -atan((2 * wheel_base_ * sin_angle) / (2 * wheel_base_ * cos_angle + track_width_ * sin_angle));
      right_wheel_angle = -atan((2 * wheel_base_ * sin_angle) / (2 * wheel_base_ * cos_angle - track_width_ * sin_angle));
    }
  }
  return std::make_pair(left_wheel_angle, right_wheel_angle);
}

std::pair<double, double> VehicleController::rear_differential_velocity()
{
  double left_vel{velocity_}, right_vel{velocity_};
  if (abs(steering_angle_) > 1e-3) {
    const double turning_radius = wheel_base_ / tan(abs(steering_angle_));
    const double vehicle_angular_vel = velocity_ / turning_radius;
    if (steering_angle_ > 0.0) {
      left_vel = vehicle_angular_vel * (turning_radius - track_width_ / 2.0);
      right_vel = vehicle_angular_vel * (turning_radius + track_width_ / 2.0);
    } else {
      left_vel = vehicle_angular_vel * (turning_radius + track_width_ / 2.0);
      right_vel = vehicle_angular_vel * (turning_radius - track_width_ / 2.0);
    }
  }
  return std::make_pair(left_vel, right_vel);
}

void VehicleController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto now = get_clock()->now();
  last_velocity_time_ = now;
  last_steering_time_ = now;

  // 1. Procesar Velocidad
  velocity_ = std::clamp(msg->linear.x, -max_velocity_, max_velocity_);

  // 2. Procesar Dirección (usamos el angular.z como ángulo de dirección directo)
  steering_angle_ = std::clamp(msg->angular.z, -max_steering_angle_, max_steering_angle_);

  // 3. Calcular Ackermann
  const auto wheel_angles = ackermann_steering_angle();
  wheel_steering_angle_ = {wheel_angles.first, wheel_angles.second};

  // 4. Calcular Diferencial y convertir a velocidad angular de rueda
  const auto wheel_vels = rear_differential_velocity();
  wheel_angular_velocity_ = {wheel_vels.first / wheel_radius_, wheel_vels.second / wheel_radius_};
}

void VehicleController::timer_callback()
{
  const auto current_time = get_clock()->now();
  if ((current_time - last_velocity_time_).nanoseconds() > timeout_duration_) {
    wheel_angular_velocity_ = {0.0, 0.0};
  }
  if ((current_time - last_steering_time_).nanoseconds() > timeout_duration_) {
    wheel_steering_angle_ = {0.0, 0.0};
  }

  std_msgs::msg::Float64MultiArray pos_msg;
  pos_msg.data = wheel_steering_angle_;
  position_publisher_->publish(pos_msg);

  std_msgs::msg::Float64MultiArray vel_msg;
  vel_msg.data = wheel_angular_velocity_;
  velocity_publisher_->publish(vel_msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleController>());
  rclcpp::shutdown();
  return 0;
}