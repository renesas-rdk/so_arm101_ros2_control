// ********************************************************************************************************************
// Copyright [2025] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
//
// The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
// and/or its licensors ("Renesas") and subject to statutory and contractual protections.
//
// Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
// display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
// purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
// SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
// NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
// INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
// OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
// be subject to different terms.
// ********************************************************************************************************************
#include "so_arm101_ros2_control/so_arm101_hardware_interface.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include "ft_servo/SCServo.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace so_arm101_ros2_control
{

hardware_interface::CallbackReturn SoArm101HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Validate joint configuration
  if (info_.joints.size() != JOINT_COUNT) {
    RCLCPP_ERROR(logger_, "Expected %zu joints, got %zu", JOINT_COUNT, info_.joints.size());
    return CallbackReturn::ERROR;
  }

  // Initialize joint state vectors
  joint_positions_.resize(JOINT_COUNT, 0.0);
  joint_velocities_.resize(JOINT_COUNT, 0.0);
  joint_position_commands_.resize(JOINT_COUNT, 0.0);
  joint_positions_prev_.resize(JOINT_COUNT, 0.0);

  // Initialize servo configuration
  servo_rotation_directions_.resize(JOINT_COUNT, 1);

  // Initialize hardware communication
  servo_driver_ = std::make_unique<SMS_STS>();
  hardware_is_connected_ = false;
  torque_enabled_ = false;

  // Validate and load parameters
  if (!validate_parameters()) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger_, "Initialized SO ARM101 interface: port=%s, baud=%d, torque_enabled=%s",
    serial_port_device_.c_str(), serial_baud_rate_, torque_enabled_on_start_ ? "true" : "false");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SoArm101HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    // Position state interface
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));

    // Velocity state interface (calculated from position differences)
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SoArm101HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn SoArm101HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating hardware interface...");

  // Initialize serial communication
  if (!servo_driver_->begin(serial_baud_rate_, serial_port_device_.c_str())) {
    RCLCPP_ERROR(
      logger_, "Failed to initialize serial communication on %s", serial_port_device_.c_str());
    return CallbackReturn::ERROR;
  }

  // Initialize all servo motors
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    const uint8_t servo_id = static_cast<uint8_t>(i + 1);
    if (!setup_servo(servo_id, i)) {
      RCLCPP_ERROR(
        logger_, "Failed to initialize servo %d for joint '%s'", servo_id,
        info_.joints[i].name.c_str());
      return CallbackReturn::ERROR;
    }
    // Small delay between servo initializations
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  hardware_is_connected_ = true;

  // Enable torque if configured
  configure_servo_torque(torque_enabled_on_start_);

  // Load calibration data if provided
  if (!calibration_file_path_.empty()) {
    if (!load_joint_calibration(calibration_file_path_)) {
      RCLCPP_WARN(logger_, "Failed to load calibration from: %s", calibration_file_path_.c_str());
    }
  }

  RCLCPP_INFO(logger_, "Hardware interface activated successfully");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SoArm101HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating hardware interface...");

  if (hardware_is_connected_) {
    // Disable servo torque for safety
    configure_servo_torque(false);
    servo_driver_->end();
    hardware_is_connected_ = false;
  }

  RCLCPP_INFO(logger_, "Hardware interface deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type SoArm101HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!hardware_is_connected_) {
    return hardware_interface::return_type::OK;
  }

  // Store previous positions for velocity calculation
  joint_positions_prev_ = joint_positions_;

  // Read positions from all servos
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    const uint8_t servo_id = static_cast<uint8_t>(i + 1);

    if (servo_driver_->FeedBack(servo_id) != -1) {
      const int servo_position = servo_driver_->ReadPos(servo_id);
      joint_positions_[i] = servo_position_to_radians(servo_position, i);
    } else {
      static auto clock = rclcpp::Clock();
      RCLCPP_WARN_THROTTLE(logger_, clock, 1000, "Failed to read from servo %d", servo_id);
    }
  }

  // Calculate velocities (rad/s) from position differences
  const double dt = period.seconds();
  if (dt > 0.0) {
    for (size_t i = 0; i < JOINT_COUNT; ++i) {
      joint_velocities_[i] = (joint_positions_[i] - joint_positions_prev_[i]) / dt;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SoArm101HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!hardware_is_connected_ || !torque_enabled_) {
    return hardware_interface::return_type::OK;
  }

  // Send position commands to all servos
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    const uint8_t servo_id = static_cast<uint8_t>(i + 1);
    const int servo_position = radians_to_servo_position(joint_position_commands_[i], i);

    if (!servo_driver_->RegWritePosEx(
          servo_id, servo_position, SERVO_SPEED_DEFAULT, SERVO_ACCELERATION_DEFAULT)) {
      static auto clock = rclcpp::Clock();
      RCLCPP_WARN_THROTTLE(logger_, clock, 1000, "Failed to send command to servo %d", servo_id);
    }
  }

  // Execute all buffered commands
  servo_driver_->RegWriteAction();
  return hardware_interface::return_type::OK;
}

// Convert between hardware-specific units and ROS standard units
double SoArm101HardwareInterface::servo_position_to_radians(
  int servo_position, size_t joint_index) const
{
  // Convert servo position (0-4095) to radians (-π to π)
  const double normalized = (static_cast<double>(servo_position) - SERVO_POSITION_CENTER) /
                            static_cast<double>(SERVO_POSITION_CENTER);  // -1 to 1
  return normalized * M_PI * servo_rotation_directions_[joint_index];
}

int SoArm101HardwareInterface::radians_to_servo_position(double radians, size_t joint_index) const
{
  // Convert radians (-π to π) to servo position (0-4095)
  const double normalized = (radians * servo_rotation_directions_[joint_index]) / M_PI;  // -1 to 1
  const int servo_position =
    static_cast<int>(normalized * SERVO_POSITION_CENTER + SERVO_POSITION_CENTER);
  return std::max(0, std::min(SERVO_POSITION_MAX, servo_position));
}

bool SoArm101HardwareInterface::setup_servo(uint8_t servo_id, size_t joint_index)
{
  // Check servo communication
  if (servo_driver_->Ping(servo_id) == -1) {
    RCLCPP_ERROR(
      logger_, "Servo %d not responding during setup (joint '%s')", servo_id,
      info_.joints[joint_index].name.c_str());
    return false;
  }

  // Configure servo for position control mode
  if (!servo_driver_->Mode(servo_id, 0)) {
    RCLCPP_ERROR(
      logger_, "Failed to set position control mode for servo %d (joint '%s')", servo_id,
      info_.joints[joint_index].name.c_str());
    return false;
  }

  // Read current position and initialize command
  if (servo_driver_->FeedBack(servo_id) != -1) {
    const int current_position = servo_driver_->ReadPos(servo_id);
    if (current_position >= 0 && current_position <= SERVO_POSITION_MAX) {
      joint_positions_[joint_index] = servo_position_to_radians(current_position, joint_index);
      joint_position_commands_[joint_index] = joint_positions_[joint_index];
      joint_positions_prev_[joint_index] = joint_positions_[joint_index];

      RCLCPP_INFO(
        logger_, "Servo %d (%s) initialized at position %d (%.3f rad)", servo_id,
        info_.joints[joint_index].name.c_str(), current_position, joint_positions_[joint_index]);
    } else {
      RCLCPP_WARN(
        logger_, "Servo %d returned invalid position %d, using center position", servo_id,
        current_position);
      joint_positions_[joint_index] = 0.0;
      joint_position_commands_[joint_index] = 0.0;
      joint_positions_prev_[joint_index] = 0.0;
    }
  } else {
    RCLCPP_WARN(
      logger_, "Failed to read initial position from servo %d, using center position", servo_id);
    joint_positions_[joint_index] = 0.0;
    joint_position_commands_[joint_index] = 0.0;
    joint_positions_prev_[joint_index] = 0.0;
  }

  return true;
}

void SoArm101HardwareInterface::configure_servo_torque(bool enable)
{
  if (!hardware_is_connected_) {
    return;
  }

  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    const uint8_t servo_id = static_cast<uint8_t>(i + 1);
    if (enable) {
      servo_driver_->Mode(servo_id, 0);  // Ensure position mode when enabling torque
    } else {
      servo_driver_->Mode(servo_id, 2);  // Ensure Idle mode when disabling torque
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    servo_driver_->EnableTorque(servo_id, enable ? 1 : 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Wait a moment for torque state to stabilize
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  torque_enabled_ = enable;

  RCLCPP_INFO(logger_, "Servo torque %s for all joints", enable ? "enabled" : "disabled");
}

bool SoArm101HardwareInterface::validate_parameters()
{
  // Parse hardware parameters with defaults
  const auto & params = info_.hardware_parameters;

  // Torque enable parameter (case-insensitive)
  auto it = params.find("enable_torque");
  if (it != params.end()) {
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    torque_enabled_on_start_ = (value == "true" || value == "1");
  } else {
    torque_enabled_on_start_ = true;
  }

  // Serial port parameter
  it = params.find("serial_port");
  serial_port_device_ = (it != params.end()) ? it->second : "/dev/ttyACM0";

  // Serial baud rate parameter
  it = params.find("serial_baudrate");
  if (it != params.end()) {
    try {
      serial_baud_rate_ = std::stoi(it->second);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_, "Invalid serial_baudrate parameter '%s': %s", it->second.c_str(), e.what());
      return false;
    }
  } else {
    serial_baud_rate_ = 1000000;  // Default baud rate
  }

  // Calibration file parameter (optional)
  it = params.find("calibration_file");
  calibration_file_path_ = (it != params.end()) ? it->second : "";

  // Validate baud rate range
  if (serial_baud_rate_ <= 0 || serial_baud_rate_ > 10000000) {
    RCLCPP_ERROR(
      logger_, "Invalid baud rate: %d (must be > 0 and <= 10,000,000)", serial_baud_rate_);
    return false;
  }

  return true;
}

bool SoArm101HardwareInterface::load_joint_calibration(const std::string & calibration_file_path)
{
  try {
    YAML::Node config = YAML::LoadFile(calibration_file_path);

    if (!config["calibration"]) {
      RCLCPP_ERROR(logger_, "Calibration file missing 'calibration' section");
      return false;
    }

    for (const auto & joint_config : config["calibration"]) {
      const std::string joint_name = joint_config.first.as<std::string>();

      ServoCalibration calibration;
      calibration.min_position = joint_config.second["min_ticks"].as<int>();
      calibration.center_position = joint_config.second["center_ticks"].as<int>();
      calibration.max_position = joint_config.second["max_ticks"].as<int>();
      calibration.position_range = calibration.max_position - calibration.min_position;

      joint_calibrations_[joint_name] = calibration;

      RCLCPP_INFO(
        logger_, "Loaded calibration for joint '%s': [%d, %d, %d]", joint_name.c_str(),
        calibration.min_position, calibration.center_position, calibration.max_position);
    }

    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_, "Failed to load calibration from '%s': %s", calibration_file_path.c_str(), e.what());
    return false;
  }
}

double SoArm101HardwareInterface::normalize_position(
  const std::string & joint_name, int ticks) const
{
  auto it = joint_calibrations_.find(joint_name);
  if (it != joint_calibrations_.end()) {
    const auto & calibration = it->second;
    const double normalized = (static_cast<double>(ticks) - calibration.center_position) /
                              (calibration.position_range / 2.0);
    return std::max(-1.0, std::min(1.0, normalized)) * M_PI;
  }
  // Fallback to default conversion if no calibration found
  return servo_position_to_radians(ticks, 0);
}

}  // namespace so_arm101_ros2_control

// Register the hardware interface as a plugin
PLUGINLIB_EXPORT_CLASS(
  so_arm101_ros2_control::SoArm101HardwareInterface, hardware_interface::SystemInterface)