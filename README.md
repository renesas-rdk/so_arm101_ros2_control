# SO ARM101 ROS2 Control

This package provides ROS 2 control hardware interface for the SO ARM101 6-DOF robotic arm using STS3215 servo motors.

## Package Contents

- `include/`: Header files including hardware interface and ft_servo driver
- `src/`: Implementation of the hardware interface
- `config/`: Controller configurations and calibration files
- `urdf/`: ROS2 control xacro macro definitions

## Hardware Interface

The `SoArm101HardwareInterface` class implements the `hardware_interface::SystemInterface` to communicate with the STS3215 servo motors via serial communication using the ft_servo driver library.

### Features

- **Position Control**: Joint position command and state interfaces
- **Calibration Support**: Load calibration data from YAML files
- **Real-time Communication**: Direct serial communication with servo motors

## Controllers

The package is configured to work with:

- **Joint Trajectory Controller**: For coordinated arm movement (5 DOF arm joints)
- **Joint Group Position Controller**: For gripper control (1 DOF gripper joint)

## Configuration Files

### Controllers (`so_arm101_controllers.yaml`)
- `so_arm101_arm_controller`: JointTrajectoryController for arm joints
- `so_arm101_gripper_controller`: JointGroupPositionController for gripper
- `joint_state_broadcaster`: Joint state publisher

### Initial Positions (`initial_positions.yaml`)
Default joint positions at startup (all zeros for SO ARM101)

### Calibration (`calibration.yaml`)
Joint-specific calibration data mapping servo ticks to joint limits

## Usage

### Including in Robot Description

```xml
<xacro:include filename="$(find so_arm101_ros2_control)/urdf/so_arm101_macro.ros2_control.xacro" />

<!-- Add ros2_control to your robot -->
<xacro:so_arm101_ros2_control
  name="so_arm101"
  initial_positions_file="$(find so_arm101_ros2_control)/config/initial_positions.yaml"
  use_fake_hardware="false" />
```

### Hardware Parameters

- `enable_torque`: Enable motor torque on startup (default: true)
- `serial_port`: Serial port device (default: /dev/ttyACM0)
- `serial_baudrate`: Communication baud rate (default: 1000000)
- `calibration_file`: Path to calibration YAML file

### Services

No services are provided by the hardware interface. Torque control is managed through the `enable_torque` hardware parameter.

## Dependencies

- `rclcpp`
- `rclcpp_lifecycle`
- `hardware_interface`
- `pluginlib`
- `yaml-cpp`

## Hardware Requirements

- SO ARM101 robotic arm with STS3215 servo motors
- USB-to-Serial converter (typically /dev/ttyACM0)
- Serial connection configured at 1000000 baud

## Servo Motor Mapping

| Joint | Servo ID | Description |
|-------|----------|-------------|
| shoulder_pan | 1 | Base rotation |
| shoulder_lift | 2 | Shoulder elevation |
| elbow_flex | 3 | Elbow flexion |
| wrist_flex | 4 | Wrist flexion |
| wrist_roll | 5 | Wrist rotation |
| gripper | 6 | Gripper open/close |

## Troubleshooting

1. **No servo response**: Check serial connection and baud rate
2. **Permission denied**: Ensure user has access to serial port (`sudo usermod -a -G dialout $USER`)
3. **Calibration issues**: Update calibration.yaml with actual servo limits
4. **Communication timeout**: Check servo IDs and wiring