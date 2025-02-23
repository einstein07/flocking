#include "flocking.h"

#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace argos3_ros2_bridge::msg;


/****************************************/
/*    Parameter Loading Functions       */
/****************************************/

void WheelTurningParams::load_from_parameters(std::shared_ptr<rclcpp::Node> node, const std::string & ns) {
  node->declare_parameter(ns + ".hard_turn_on_angle_threshold", hard_turn_on_angle_threshold);
  node->declare_parameter(ns + ".soft_turn_on_angle_threshold", soft_turn_on_angle_threshold);
  node->declare_parameter(ns + ".no_turn_angle_threshold", no_turn_angle_threshold);
  node->declare_parameter(ns + ".max_speed", max_speed);

  node->get_parameter(ns + ".hard_turn_on_angle_threshold", hard_turn_on_angle_threshold);
  hard_turn_on_angle_threshold = hard_turn_on_angle_threshold * M_PI / 180.0;
  node->get_parameter(ns + ".soft_turn_on_angle_threshold", soft_turn_on_angle_threshold);
  soft_turn_on_angle_threshold = soft_turn_on_angle_threshold * M_PI / 180.0;
  node->get_parameter(ns + ".no_turn_angle_threshold", no_turn_angle_threshold);
  no_turn_angle_threshold = no_turn_angle_threshold * M_PI / 180.0;
  node->get_parameter(ns + ".max_speed", max_speed);

  // Initialize the turning mechanism to NO_TURN.
  turning_mechanism = TurningMechanism::NO_TURN;
}

void FlockingInteractionParams::load_from_parameters(std::shared_ptr<rclcpp::Node> node, const std::string & ns) {
  node->declare_parameter(ns + ".target_distance", target_distance);
  node->declare_parameter(ns + ".gain", gain);
  node->declare_parameter(ns + ".exponent", exponent);

  node->get_parameter(ns + ".target_distance", target_distance);
  node->get_parameter(ns + ".gain", gain);
  node->get_parameter(ns + ".exponent", exponent);
}

double FlockingInteractionParams::generalizedLennardJones(double distance) const {
  double normDistExp = std::pow(target_distance / distance, exponent);
  return -gain / distance * (normDistExp * normDistExp - normDistExp);
}

/****************************************/
/*       Controller Implementation      */
/****************************************/

FlockingController::FlockingController(std::shared_ptr<rclcpp::Node> node) 
{
  printed_ = false;
  this -> node_ = node;
  // Retrieve the domain ID via the NodeBaseInterface
  
  ns_ = node_->get_namespace();
  initialized_ = false;
  // Load controller parameters from the parameter server.
  wheel_params_.load_from_parameters(node_, "wheel_turning");
  flocking_params_.load_from_parameters(node_, "flocking");
  node_->declare_parameter("wheel_separation", 0.2);
  node_->get_parameter("wheel_separation", wheel_separation_);

  // Initialize subscribers for sensor topics.
  // (In a real implementation, these messages would be defined in separate packages.)
  std::stringstream lightListTopic, blobTopic, ledTopic;
    lightListTopic << ns_ << "/lightList";
    blobTopic << ns_ << "/blobList";
    ledTopic << ns_ << "/cmd_led";


  light_sub_ = node_ -> create_subscription<argos3_ros2_bridge::msg::LightList>(
    lightListTopic.str(),
    1,
    std::bind(&FlockingController::light_sensor_callback, this, _1)
    );

  camera_sub_ = node_ -> create_subscription<argos3_ros2_bridge::msg::BlobList>(
    blobTopic.str(),
    1,
    std::bind(&FlockingController::camera_sensor_callback, this, _1)
    );

  // Publisher for wheel commands (converted to a Twist message).
  cmd_pub_ = node_ -> create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  cmd_led_ = node_ -> create_publisher<argos3_ros2_bridge::msg::Led>(ledTopic.str(), 1);

  // Set up a control loop timer (e.g. 50Hz).
  timer_ = node_ -> create_wall_timer(20ms, std::bind(&FlockingController::timer_callback, this));

}

/****************************************/
/*         Sensor Callbacks             */
/****************************************/

void FlockingController::light_sensor_callback(const argos3_ros2_bridge::msg::LightList & msg) {
  current_light_msg_ = msg;
}

void FlockingController::camera_sensor_callback(const argos3_ros2_bridge::msg::BlobList & msg) {
  current_camera_msg_ = msg;
}

/****************************************/
/*         Control Loop Step            */
/****************************************/

void FlockingController::timer_callback() {
  if (!printed_) {
    auto domain_id = node_->get_node_base_interface()->get_context()->get_domain_id();
    std::cout << "Controller running on ROS_DOMAIN_ID: " << domain_id << std::endl;
    printed_ = true;
  }
  if (!initialized_ && cmd_led_ -> get_subscription_count() > 0){
    /**Led led_msg;
    led_msg.color = "red";
    led_msg.index = 12;
    led_msg.mode = "SINGLE";
    cmd_led_->publish(led_msg);*/
    initialized_ = true;
  }
  if (initialized_ && cmd_led_ -> get_subscription_count() == 0){
    auto current_time = node_ -> now();
    std::cout << "Bot: " << ns_ << " controller done. Current time: " << current_time.seconds() << " secs" << std::endl;
		rclcpp::shutdown();
		exit(0);
  }
  // Combine vectors from the light sensor and flocking interactions.
  Vector2 vector_to_light = vectorToLight();
  Vector2 flocking_vector = flockingVector();
  setWheelSpeedsFromVector(vector_to_light + flocking_vector);
}

/****************************************/
/*         Vector Calculations          */
/****************************************/

Vector2 FlockingController::vectorToLight() {
  Vector2 accum(0.0, 0.0);
  // Sum contributions from each light sensor reading (interpreted as polar vectors).
  for (const auto & reading : current_light_msg_.lights) {
    accum += Vector2(reading.value * std::cos(reading.angle),
                     reading.value * std::sin(reading.angle));
  }
  if (accum.length() > 0.0) {
    accum = accum.normalized() * (0.25 * wheel_params_.max_speed);
  }
  return accum;
}

Vector2 FlockingController::flockingVector() {
  if (current_camera_msg_.blobs.empty()) {
    return Vector2(0.0, 0.0);
  }
  Vector2 accum(0.0, 0.0);
  int count = 0;
  for (const auto & blob : current_camera_msg_.blobs) {
    // Consider only red blobs and only those closer than 180% of the target distance.
    if (blob.color == "red" &&
        blob.distance < flocking_params_.target_distance * 1.80) {
      double lj = flocking_params_.generalizedLennardJones(blob.distance);
      accum += Vector2(lj * std::cos(blob.angle), lj * std::sin(blob.angle));
      ++count;
    }
  }
  if (count > 0) {
    accum = accum / static_cast<double>(count);
    if (accum.length() > wheel_params_.max_speed) {
      accum = accum.normalized() * wheel_params_.max_speed;
    }
    return accum;
  }
  return Vector2(0.0, 0.0);
}

/****************************************/
/*     Wheel Speed Computation          */
/****************************************/

void FlockingController::setWheelSpeedsFromVector(const Vector2 & heading) {
  double heading_angle = std::atan2(heading.y, heading.x);
  //double heading_length = heading.length();
  double base_speed = wheel_params_.max_speed;//std::min(heading_length, wheel_params_.max_speed);
  double abs_angle = std::fabs(heading_angle);
  
  // State transition logic for turning mechanism.
  if (wheel_params_.turning_mechanism == TurningMechanism::HARD_TURN) {
    if (abs_angle <= wheel_params_.soft_turn_on_angle_threshold){
      wheel_params_.turning_mechanism = TurningMechanism::SOFT_TURN;
      //std::cout << "Soft turn" << std::endl;
    }
  }
  if (wheel_params_.turning_mechanism == TurningMechanism::SOFT_TURN) {
    if (abs_angle > wheel_params_.hard_turn_on_angle_threshold){
      wheel_params_.turning_mechanism = TurningMechanism::HARD_TURN;
      //std::cout << "Hard turn" << std::endl;
    }
    else if (abs_angle <= wheel_params_.no_turn_angle_threshold){
      wheel_params_.turning_mechanism = TurningMechanism::NO_TURN;
      //std::cout << "No turn" << std::endl;
    }
  }
  if (wheel_params_.turning_mechanism == TurningMechanism::NO_TURN) {
    if (abs_angle > wheel_params_.hard_turn_on_angle_threshold){
      wheel_params_.turning_mechanism = TurningMechanism::HARD_TURN;
      //std::cout << "Hard turn" << std::endl;
    }
    else if (abs_angle > wheel_params_.no_turn_angle_threshold){
      wheel_params_.turning_mechanism = TurningMechanism::SOFT_TURN;
      //std::cout << "Soft turn" << std::endl;
    }
  }

  double speed1, speed2;
  switch (wheel_params_.turning_mechanism) {
    case TurningMechanism::NO_TURN:
      speed1 = base_speed;
      speed2 = base_speed;
      break;
    case TurningMechanism::SOFT_TURN: {
      double speed_factor = (wheel_params_.hard_turn_on_angle_threshold - abs_angle) /
                            wheel_params_.hard_turn_on_angle_threshold;
      speed1 = base_speed - base_speed * (1.0 - speed_factor);
      speed2 = base_speed + base_speed * (1.0 - speed_factor);
      break;
    }
    case TurningMechanism::HARD_TURN:
      speed1 = -wheel_params_.max_speed;
      speed2 = wheel_params_.max_speed;
      break;
  }

  double left_speed, right_speed;
  if (heading_angle > 0) {
    // Turn left.
    left_speed  = speed1;
    right_speed = speed2;
  } else {
    // Turn right.
    left_speed  = speed2;
    right_speed = speed1;
  }

  geometry_msgs::msg::Twist twist;
  twist.linear.x = left_speed;
  twist.linear.y = right_speed;
  cmd_pub_->publish(twist);
  // Convert differential wheel speeds to a Twist command.
  // Using standard differential drive kinematics:
  //   v = (left_speed + right_speed) / 2
  //   ω = (right_speed - left_speed) / wheel_separation
  /**double v = (left_speed + right_speed) / 2.0;
  double omega = (right_speed - left_speed) / wheel_separation_;

  geometry_msgs::msg::Twist twist;
  twist.linear.x = v;
  twist.angular.z = omega;
  cmd_pub_->publish(twist);*/

}


/**************************
 * Main function
 *************************/
int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("argos_ros_node");
  FlockingController controller(node);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;

}

