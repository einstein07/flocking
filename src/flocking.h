#ifndef FOOTBOT_FLOCKING_CONTROLLER_HPP
#define FOOTBOT_FLOCKING_CONTROLLER_HPP
#include <cmath>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "argos3_ros2_bridge/msg/led.hpp"
#include "argos3_ros2_bridge/msg/blob_list.hpp"
#include "argos3_ros2_bridge/msg/light_list.hpp"



/* 
 * Simple 2D vector class to support polar/cartesian conversions.
 */
struct Vector2 {
  double x;
  double y;
  Vector2(double _x = 0.0, double _y = 0.0) : x(_x), y(_y) {}
  double length() const { return std::hypot(x, y); }
  Vector2 normalized() const {
    double l = length();
    return (l > 0) ? Vector2(x / l, y / l) : Vector2(0.0, 0.0);
  }
  double angle() const { return std::atan2(y, x); }
  Vector2 operator+(const Vector2& other) const {
    return Vector2(x + other.x, y + other.y);
  }
  Vector2& operator+=(const Vector2& other) {
    x += other.x;
    y += other.y;
    return *this;
  }
  Vector2 operator/(double d) const { return Vector2(x / d, y / d); }
  Vector2 operator*(double d) const { return Vector2(x * d, y * d); }
};

/*
 * Turning mechanism state.
 */
enum class TurningMechanism {
  NO_TURN = 0,
  SOFT_TURN,
  HARD_TURN
};

/*
 * Wheel turning parameters.
 */
struct WheelTurningParams {
  TurningMechanism turning_mechanism;
  double hard_turn_on_angle_threshold; // radians
  double soft_turn_on_angle_threshold; // radians
  double no_turn_angle_threshold;      // radians
  double max_speed;                    // maximum wheel speed

  WheelTurningParams()
  : turning_mechanism(TurningMechanism::NO_TURN),
    hard_turn_on_angle_threshold(0.5),
    soft_turn_on_angle_threshold(0.3),
    no_turn_angle_threshold(0.1),
    max_speed(1.0) {}

  void load_from_parameters(std::shared_ptr<rclcpp::Node> node, const std::string & ns);
};

/*
 * Flocking interaction parameters.
 */
struct FlockingInteractionParams {
  double target_distance; ///< Preferred distance between robots
  double gain;          ///< Gain for the Lennard-Jones potential
  double exponent;      ///< Exponent for the potential

  FlockingInteractionParams()
  : target_distance(1.0),
    gain(1.0),
    exponent(2.0) {}

  void load_from_parameters(std::shared_ptr<rclcpp::Node> node, const std::string & ns);
  double generalizedLennardJones(double distance) const;
};

/*
 * The ROS2 controller node for foot-bot flocking.
 */
class FlockingController {
public:
  FlockingController(std::shared_ptr<rclcpp::Node> node);

private:
  std::shared_ptr<rclcpp::Node> node_;
  const char* ns_;
  bool initialized_leds_;
  // Timer callback for the control loop.
  void timer_callback();

  // Sensor callbacks to update the latest sensor data.
  void light_sensor_callback(const argos3_ros2_bridge::msg::LightList & msg);
  void camera_sensor_callback(const argos3_ros2_bridge::msg::BlobList & msg);

  // Control functions analogous to the ARGoS controller.
  Vector2 vectorToLight();
  Vector2 flockingVector();
  void setWheelSpeedsFromVector(const Vector2 & heading);

  // Subscribers and publisher.
  rclcpp::Subscription<argos3_ros2_bridge::msg::LightList>::SharedPtr light_sub_;
  rclcpp::Subscription<argos3_ros2_bridge::msg::BlobList>::SharedPtr camera_sub_;
  rclcpp::Publisher<argos3_ros2_bridge::msg::Led>::SharedPtr cmd_led_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Latest sensor data.
  argos3_ros2_bridge::msg::LightList current_light_msg_;
  argos3_ros2_bridge::msg::BlobList current_camera_msg_;

  // Controller parameters.
  WheelTurningParams wheel_params_;
  FlockingInteractionParams flocking_params_;
  double wheel_separation_; // Distance between wheels for kinematics.
};


#endif  // FOOTBOT_FLOCKING_CONTROLLER_HPP
