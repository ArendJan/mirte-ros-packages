// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mirte_control/mirte_dart_control.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mirte_dart_control {

  std::string convert_to_snake_case(const std::string &input) {
  std::string result = "";

  for (auto it = input.cbegin(); it != input.cend(); ++it) {
    if (std::isupper(*it) && !result.empty()) {
      result.push_back('_');
    }
    result.push_back(std::tolower(*it));
  }

  return result;
}

hardware_interface::CallbackReturn MirteDartHWInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  node =
      rclcpp::Node::make_shared(convert_to_snake_case(get_name()));
  auto logger_name = std::string(node->get_namespace()).substr(1) + "." +
                     std::string(node->get_name());
  logger_ = rclcpp::get_logger(logger_name);

  //Define service client paths
  auto steering_service = "/io/servo/stuur/set_angle";
  auto throttle_service = "/io/servo/gas/set_angle";

  auto encoder_topic = "io/encoder/main";
  auto imu_topic = "io/imu/movement/data";
  
  // Initialize the Encoder Subscriber
  encoder_sub_ = node->create_subscription<mirte_msgs::msg::Encoder>(
    encoder_topic,
    10, // QoS history depth
    [this](const mirte_msgs::msg::Encoder::SharedPtr msg) {
      // This runs instantly in the background whenever a packet arrives
      this->latest_encoder_ticks_ = static_cast<double>(msg->value); 
    });

  // Initialize the IMU Subscriber
  imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic,
    10,
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      this->latest_imu_yaw_rate_ = msg->angular_velocity.z;
    });

  steering_client_ =
      node->create_client<mirte_msgs::srv::SetServoAngle>(steering_service);
  
  throttle_client_ =
      node->create_client<mirte_msgs::srv::SetServoAngle>(throttle_service);
 
  while (!steering_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_.value(),
                   "Interrupted while waiting for the service. Exiting.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
        logger_.value(),
        "service io/servo/stuur/set_angle not available, waiting again...");
  }
 
  // TODO: combine with above
  while (!throttle_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_.value(),
                   "Interrupted while waiting for the service. Exiting.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
        logger_.value(),
        "service io/servo/gas/set_angle not available, waiting again...");
  }

  // Check if the number of joints is correct based on the mode of operation
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("MirteDartHWInterface"),
      "MirteDartHWInterface::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("front") != std::string::npos;

    // Steering joints have a position command interface and a position state interface
    if (joint_is_steering)
    {
      steering_joint_ = joint.name;
      RCLCPP_INFO(rclcpp::get_logger("MirteDartHWInterface"), "Joint '%s' is a steering joint.", joint.name.c_str());

      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("MirteDartHWInterface"), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("MirteDartHWInterface"), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("MirteDartHWInterface"), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("MirteDartHWInterface"), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("MirteDartHWInterface"), "Joint '%s' is a drive joint.", joint.name.c_str());
      traction_joint_ = joint.name;

      // Drive joints have a velocity command interface and a velocity state interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("MirteDartHWInterface"), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("MirteDartHWInterface"), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("MirteDartHWInterface"), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("MirteDartHWInterface"), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("MirteDartHWInterface"), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production
  // code
  // hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  // hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Newly added for Humble
std::vector<hardware_interface::StateInterface> MirteDartHWInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    steering_joint_, hardware_interface::HW_IF_POSITION, &steering_pos_state_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    traction_joint_, hardware_interface::HW_IF_POSITION, &traction_pos_state_));
    
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    traction_joint_, hardware_interface::HW_IF_VELOCITY, &traction_vel_state_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MirteDartHWInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    steering_joint_, hardware_interface::HW_IF_POSITION, &steering_pos_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    traction_joint_, hardware_interface::HW_IF_VELOCITY, &traction_vel_cmd_));

  return command_interfaces;
}
//Newly added for Humble

hardware_interface::CallbackReturn MirteDartHWInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MirteDartHWInterface"), "Configuring ...please wait...");

  // for (auto i = 0; i < hw_start_sec_; i++)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(rclcpp::get_logger("MirteDartHWInterface"), "%.1f seconds left...", hw_start_sec_ - i);
  // }

  // reset values always when configuring hardware
  steering_pos_state_ = 0.0;
  steering_pos_cmd_ = 0.0;
  traction_vel_state_ = 0.0;
  traction_vel_cmd_ = 0.0;
  traction_pos_state_ = 0.0;

  RCLCPP_INFO(rclcpp::get_logger("MirteDartHWInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MirteDartHWInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MirteDartHWInterface"), "Activating ...please wait...");

  // for (auto i = 0; i < hw_start_sec_; i++)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(rclcpp::get_logger("MirteDartHWInterface"), "%.1f seconds left...", hw_start_sec_ - i);
  // }

  // command and state should be equal when starting
  steering_pos_cmd_ = steering_pos_state_;
  traction_vel_cmd_ = traction_vel_state_;

  RCLCPP_INFO(rclcpp::get_logger("MirteDartHWInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MirteDartHWInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("MirteDartHWInterface"), "Deactivating ...please wait...");

  // for (auto i = 0; i < hw_stop_sec_; i++)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(rclcpp::get_logger("MirteDartHWInterface"), "%.1f seconds left...", hw_stop_sec_ - i);
  // }
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("MirteDartHWInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MirteDartHWInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  //1. Process any incoming service responses from the network queue
  if (node) {
    rclcpp::spin_some(node);
  }

  // 2. CLOSED-LOOP MATH (No service requests needed here anymore!)
  const double TICKS_TO_RAD = 0.17699; 
  const double TICKS_TO_METERS = 0.00412708;
  static double last_encoder_ticks = 0.0;

  // Handle initialization on the first frame
  if (last_encoder_ticks == 0.0 && latest_encoder_ticks_ != 0.0) {
    last_encoder_ticks = latest_encoder_ticks_;
  }

  // Calculate changes since the last frame
  double delta_ticks = latest_encoder_ticks_ - last_encoder_ticks;
  last_encoder_ticks = latest_encoder_ticks_;
  double delta_distance = delta_ticks * TICKS_TO_RAD;

  // Sync state variables for ros2_control
  
  // steering_pos_state_ = steering_pos_cmd_;
  traction_pos_state_ += delta_distance;
  double traction_pos_state_meters = traction_pos_state_*TICKS_TO_METERS;

  if (period.seconds() > 0.0) {
    traction_vel_state_ = delta_distance / period.seconds();
  } else {
    traction_vel_state_ = 0.0;
  }
  // --- 1. COLLECT SENSOR INPUTS ---
  const double L = 0.176;              // Your physical wheelbase in meters
  double imu_yaw_rate = latest_imu_yaw_rate_; // From your IMU subscriber topic

  // Get your linear velocity in meters per second (m/s)
  // (Assuming traction_vel_state_ is already calculated in rad/s from your encoders)
  const double WHEEL_RADIUS = 0.0233;
  double forward_velocity = traction_vel_state_ * WHEEL_RADIUS; 

  // --- 2. THE DIRECT KINEMATIC RECONSTRUCTION ---
  // We enforce a tiny velocity threshold. If the robot is truly stopped, 
  // any IMU value is pure noise, so we drop it to zero.
  if (std::abs(forward_velocity) > 0.02) {

    // Calculate steering angle based entirely on how fast the chassis is rotating
    double calculated_phi = std::atan2(imu_yaw_rate * L, forward_velocity);

    // Apply a strict physical clamp so noise doesn't spike past your steering limits
    // 0.52 radians is roughly 30 degrees maximum steering lock
    const double MAX_STEER_LIMIT = 0.52; 
    steering_pos_state_ = std::clamp(calculated_phi, -MAX_STEER_LIMIT, MAX_STEER_LIMIT);

  } else {
    // Standing still fallback: hold the wheel at 0 or trust your last command frame
    steering_pos_state_ = 0.0; 
  }
  
  std::stringstream ss;
  ss << "Reading states:" << std::fixed << std::setprecision(2) << std::endl

     << "\t" << "measured yaw: " << latest_imu_yaw_rate_ << " rad" << std::endl
     << "\t" << "Encoder measured: " << delta_distance << " m" << std::endl
     << "\t" << "Steering Pos: " << steering_pos_state_ << " rad" << std::endl
     << "\t" << "Traction Pos: " << traction_pos_state_meters << " m" << std::endl
     << "\t" << "Traction Vel: " << traction_vel_state_ << " m/s";

  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("MirteDartHWInterface"), 
    steady_clock, 
    500, 
    "%s", 
    ss.str().c_str()
  );
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MirteDartHWInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;

  ss << "Writing commands:" << std::fixed << std::setprecision(2) << std::endl
     << "\t" << "Steering Cmd (Pos): " << steering_pos_cmd_ << std::endl
     << "\t" << "Traction Cmd (Vel): " << traction_vel_cmd_;

  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("MirteDartHWInterface"), 
    steady_clock, 
    500, 
    "%s", 
    ss.str().c_str()
  );
  // END: This part here is for exemplary purposes - Please do not copy to your production code

   // BEGIN: Code written for MIRTE-on-DART
  auto steering_request =
      std::make_shared<mirte_msgs::srv::SetServoAngle::Request>();
  double position_degrees = steering_pos_cmd_*180/M_PI;

  int steering_angle =
      int(std::max(std::min(float(position_degrees) + 90.0, 111.0), 69.0)); // from tests it became clear that 92 is approximately straight
  if (steering_angle != last_cmd_steering_) {
    steering_request->angle = float(steering_angle);
    auto result = steering_client_->async_send_request(steering_request);
    last_cmd_steering_ = steering_angle;
  }
 
  auto throttle_request =
      std::make_shared<mirte_msgs::srv::SetServoAngle::Request>();
  double velocity = traction_vel_cmd_*0.0233; //radius of the wheels
  float velocity_factor_forward = 8.26f; //this factor was deduced from tests
  float velocity_factor_backward = 10.0f;//this factor was deduced from tests

  int throttle_angle = 90;//anders bestaat throttle_angle alleen binnen de {} hieronder

  if (velocity>0) { //forward drive
    throttle_angle =
    std::clamp(int(velocity * velocity_factor_forward + 92.56), 0, 180); // value deduced from tests
  }
  if (velocity<0) { //backward drive
    throttle_angle =
    std::clamp(int(velocity * velocity_factor_backward + 82.8), 0, 180); //value deduced from tests
  }
  if (velocity==0) {
    throttle_angle=92; // value deduced from tests
  }

  //int throttle_angle =
  //  std::clamp(int(velocity * velocity_factor + 90), 0, 180);
  if (throttle_angle != last_cmd_throttle_) {
    throttle_request->angle = float(throttle_angle);
    auto result = throttle_client_->async_send_request(throttle_request);
    last_cmd_throttle_ = throttle_angle;
  }
 

  
  // END: Code written for MIRTE-on-DART

  return hardware_interface::return_type::OK;
}

}  // namespace mirte_dart_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mirte_dart_control::MirteDartHWInterface, hardware_interface::SystemInterface)