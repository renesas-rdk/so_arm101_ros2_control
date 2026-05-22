// ********************************************************************************************************************
// Copyright [2026] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
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

#pragma once

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "so_arm101_ros2_control/visibility_control.hpp"

// Forward declaration for SMS_STS servo driver
class SMS_STS;

namespace so_arm101_ros2_control
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * Hardware interface for SO ARM101 6-DOF robotic arm with STS3215 servo motors
 *
 * This class implements the ROS2 hardware interface for the SO ARM101 robotic arm,
 * providing position control for 6 joints via serial communication with STS3215 servos.
 * It supports real-time position control and velocity feedback calculation.
 */
class SoArm101HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SoArm101HardwareInterface)

  SO_ARM101_ROS2_CONTROL_PUBLIC
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  SO_ARM101_ROS2_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SO_ARM101_ROS2_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  SO_ARM101_ROS2_CONTROL_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  SO_ARM101_ROS2_CONTROL_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  SO_ARM101_ROS2_CONTROL_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SO_ARM101_ROS2_CONTROL_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Hardware configuration constants
  static constexpr size_t JOINT_COUNT = 6;
  static constexpr int SERVO_SPEED_DEFAULT = 4500;
  static constexpr int SERVO_ACCELERATION_DEFAULT = 255;
  static constexpr int SERVO_POSITION_CENTER_DEFAULT = 2048;  // Default center when no calibration
  static constexpr int SERVO_POSITION_MAX = 4095;

  // Unit conversion methods
  double convert_ticks_to_radians(int ticks, size_t joint_index) const;
  int convert_radians_to_ticks(double radians, size_t joint_index) const;

  // Calibration data helper methods
  int get_center_position(size_t joint_index) const;
  int get_min_position(size_t joint_index) const;
  int get_max_position(size_t joint_index) const;
  bool is_position_within_limits(int position, size_t joint_index) const;

  // Hardware setup and configuration
  bool setup_servo(uint8_t servo_id, size_t joint_index);
  void configure_servo_torque(bool enable);
  bool validate_parameters();

  // Calibration methods
  bool load_joint_calibration(const std::string & calibration_file_path);

  // Joint state data (ROS2 interface)
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_position_commands_;
  std::vector<double> joint_positions_prev_;  // For velocity calculation

  // GPIO interface for arm administrative control
  double arm_torque_enable_state_;
  double arm_torque_enable_command_;

  // Hardware communication
  std::unique_ptr<SMS_STS> servo_driver_;

  // Logging
  rclcpp::Logger logger_{rclcpp::get_logger("SoArm101HardwareInterface")};

  // Configuration parameters
  std::string serial_port_device_;
  int serial_baud_rate_;
  bool torque_enabled_on_start_;
  std::string calibration_file_path_;

  // Runtime state
  bool hardware_is_connected_;
  bool torque_enabled_;

  // Joint calibration data structure
  struct ServoCalibration
  {
    bool has_calibration = false;  // Whether this joint has calibration data
    int min_ticks;                 // Minimum safe position in raw servo ticks
    int max_ticks;                 // Maximum safe position in raw servo ticks
    int center_ticks;              // Center position in raw servo ticks - corresponds to 0 radians
  };
  std::vector<ServoCalibration> joint_calibrations_;
};

}  // namespace so_arm101_ros2_control