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
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
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

  // Initialize calibration vector
  joint_calibrations_.resize(JOINT_COUNT);

  // Initialize hardware communication
  servo_driver_ = std::make_unique<SMS_STS>();
  hardware_is_connected_ = false;
  torque_enabled_ = false;

  // Validate and load parameters
  if (!validate_parameters()) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger_,
    "Initialized SO ARM101 interface: port=%s, baud=%d, torque_enabled=%s, calibration_file=%s",
    serial_port_device_.c_str(), serial_baud_rate_, torque_enabled_on_start_ ? "true" : "false",
    calibration_file_path_.empty() ? "none" : calibration_file_path_.c_str());

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

  hardware_is_connected_ = true;

  // Load calibration data before servo initialization if provided
  if (!calibration_file_path_.empty()) {
    if (!load_joint_calibration(calibration_file_path_)) {
      RCLCPP_WARN(logger_, "Failed to load calibration from: %s", calibration_file_path_.c_str());
    }
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

  // Enable torque if configured
  configure_servo_torque(torque_enabled_on_start_);

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
      joint_positions_[i] = convert_ticks_to_radians(servo_position, i);
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
    const int servo_position = convert_radians_to_ticks(joint_position_commands_[i], i);

    // Validate position is within limits
    if (!is_position_within_limits(servo_position, i)) {
      static auto clock = rclcpp::Clock();
      RCLCPP_WARN_THROTTLE(
        logger_, clock, 1000,
        "Position command for joint %zu (%.3f rad -> %d ticks) exceeds limits [%d, %d]", i,
        joint_position_commands_[i], servo_position, get_min_position(i), get_max_position(i));
    }

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

// Simplified conversion methods using joint index
double SoArm101HardwareInterface::convert_ticks_to_radians(int ticks, size_t joint_index) const
{
  const int center_pos = get_center_position(joint_index);
  const int offset_from_center = ticks - center_pos;

  // Convert to radians: 4096 ticks = 2π radians
  // So 1 tick = 2π/4096 radians
  return (static_cast<double>(offset_from_center) * 2.0 * M_PI) / 4096.0;
}

int SoArm101HardwareInterface::convert_radians_to_ticks(double radians, size_t joint_index) const
{
  const int center_pos = get_center_position(joint_index);

  // Convert radians to ticks: 4096 ticks = 2π radians
  // So 1 radian = 4096/(2π) ticks
  const int offset_from_center = static_cast<int>((radians * 4096.0) / (2.0 * M_PI));
  const int ticks = center_pos + offset_from_center;

  // Clamp to valid range
  return std::max(get_min_position(joint_index), std::min(get_max_position(joint_index), ticks));
}

// Helper methods for calibration data access
int SoArm101HardwareInterface::get_center_position(size_t joint_index) const
{
  if (
    joint_index < joint_calibrations_.size() && joint_calibrations_[joint_index].has_calibration) {
    return joint_calibrations_[joint_index].center_ticks;
  }
  return SERVO_POSITION_CENTER_DEFAULT;
}

int SoArm101HardwareInterface::get_min_position(size_t joint_index) const
{
  if (
    joint_index < joint_calibrations_.size() && joint_calibrations_[joint_index].has_calibration) {
    return joint_calibrations_[joint_index].min_ticks;
  }
  return 0;
}

int SoArm101HardwareInterface::get_max_position(size_t joint_index) const
{
  if (
    joint_index < joint_calibrations_.size() && joint_calibrations_[joint_index].has_calibration) {
    return joint_calibrations_[joint_index].max_ticks;
  }
  return SERVO_POSITION_MAX;
}

bool SoArm101HardwareInterface::is_position_within_limits(int position, size_t joint_index) const
{
  return position >= get_min_position(joint_index) && position <= get_max_position(joint_index);
}

bool SoArm101HardwareInterface::setup_servo(uint8_t servo_id, size_t joint_index)
{
  const std::string & joint_name = info_.joints[joint_index].name;

  // Check servo communication
  if (servo_driver_->Ping(servo_id) == -1) {
    RCLCPP_ERROR(
      logger_, "Servo %d not responding during setup (joint '%s')", servo_id, joint_name.c_str());
    return false;
  }

  // Configure servo for position control mode
  if (!servo_driver_->Mode(servo_id, 0)) {
    RCLCPP_ERROR(
      logger_, "Failed to set position control mode for servo %d (joint '%s')", servo_id,
      joint_name.c_str());
    return false;
  }

  // Read current position and initialize command
  if (servo_driver_->FeedBack(servo_id) != -1) {
    const int current_position = servo_driver_->ReadPos(servo_id);

    if (current_position >= 0 && current_position <= SERVO_POSITION_MAX) {
      // Convert current position to radians
      joint_positions_[joint_index] = convert_ticks_to_radians(current_position, joint_index);
      joint_position_commands_[joint_index] = joint_positions_[joint_index];
      joint_positions_prev_[joint_index] = joint_positions_[joint_index];

      // Check if position is within calibrated limits
      if (!is_position_within_limits(current_position, joint_index)) {
        RCLCPP_WARN(
          logger_, "Servo %d (%s) current position %d is outside calibrated limits [%d, %d]",
          servo_id, joint_name.c_str(), current_position, get_min_position(joint_index),
          get_max_position(joint_index));
      }

      RCLCPP_INFO(
        logger_, "Servo %d (%s) initialized at position %d (%.3f rad)", servo_id,
        joint_name.c_str(), current_position, joint_positions_[joint_index]);
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

    size_t calibration_count = 0;
    for (const auto & joint_config : config["calibration"]) {
      const std::string joint_name = joint_config.first.as<std::string>();
      const YAML::Node & joint_data = joint_config.second;

      // Find the joint index by name
      size_t joint_index = JOINT_COUNT;  // Invalid index initially
      for (size_t i = 0; i < JOINT_COUNT; ++i) {
        if (info_.joints[i].name == joint_name) {
          joint_index = i;
          break;
        }
      }

      if (joint_index >= JOINT_COUNT) {
        RCLCPP_WARN(logger_, "Calibration for unknown joint '%s' ignored", joint_name.c_str());
        continue;
      }

      // Set calibration data for this joint index
      joint_calibrations_[joint_index].has_calibration = true;
      joint_calibrations_[joint_index].min_ticks = joint_data["min_ticks"].as<int>();
      joint_calibrations_[joint_index].max_ticks = joint_data["max_ticks"].as<int>();
      joint_calibrations_[joint_index].center_ticks = joint_data["center_ticks"].as<int>();

      RCLCPP_INFO(
        logger_, "Loaded calibration for joint %zu ('%s'): min=%d, center=%d, max=%d", joint_index,
        joint_name.c_str(), joint_calibrations_[joint_index].min_ticks,
        joint_calibrations_[joint_index].center_ticks, joint_calibrations_[joint_index].max_ticks);

      calibration_count++;
    }

    RCLCPP_INFO(logger_, "Successfully loaded calibration for %zu joints", calibration_count);
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_, "Failed to load calibration from '%s': %s", calibration_file_path.c_str(), e.what());
    return false;
  }
}

}  // namespace so_arm101_ros2_control

// Register the hardware interface as a plugin
PLUGINLIB_EXPORT_CLASS(
  so_arm101_ros2_control::SoArm101HardwareInterface, hardware_interface::SystemInterface)