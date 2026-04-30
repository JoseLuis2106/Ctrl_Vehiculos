#ifndef VEHICLE_CONTROLLER_HPP
#define VEHICLE_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp" // Incluido para el soporte de /cmd_vel

class VehicleController : public rclcpp::Node
{

public:
  VehicleController(const double timer_period = 1e-2, const double timeout_duration = 1e9);

private:
  /**
   * @brief Calcula los ángulos de dirección Ackermann para las ruedas izquierda y derecha.
   */
  std::pair<double, double> ackermann_steering_angle();

  /**
   * @brief Calcula las velocidades del diferencial trasero.
   */
  std::pair<double, double> rear_differential_velocity();

  /**
   * @brief Timer para publicar los comandos y verificar timeouts.
   */
  void timer_callback();

  /**
   * @brief Callback unificado para recibir comandos de velocidad y dirección.
   * * @param msg Mensaje Twist con linear.x (velocidad) y angular.z (ángulo de dirección).
   */
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  double timeout_duration_;
  rclcpp::Time last_velocity_time_;
  rclcpp::Time last_steering_time_;

  double body_width_;
  double body_length_;
  double wheel_radius_;
  double wheel_width_;
  double max_steering_angle_;
  double max_velocity_;
  double wheel_base_;
  double track_width_;

  double steering_angle_;
  double velocity_;

  std::vector<double> wheel_angular_velocity_;
  std::vector<double> wheel_steering_angle_;

  // Suscriptor unificado para cmd_vel
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // VEHICLE_CONTROLLER_HPP