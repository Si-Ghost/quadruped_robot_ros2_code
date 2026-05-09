#ifndef UNITREE_HARDWARE_INTERFACE__UNITREE_HARDWARE_INTERFACE_HPP
#define UNITREE_HARDWARE_INTERFACE__UNITREE_HARDWARE_INTERFACE_HPP

#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <future>
#include <mutex>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace unitree_hardware_interface
{

static constexpr int TOTAL_MOTORS  = 3;   // TODO: restore to 12
static constexpr int PORTS         = 1;   // TODO: restore to 4
static constexpr int MOTORS_PER_PORT = 3;

struct MotorPort
{
  std::string              path;
  int                      base;           // global index offset
  std::unique_ptr<SerialPort> serial;
  std::vector<MotorCmd>    cmds;           // 3 per port
  std::vector<MotorData>   data;           // 3 per port
  int                      motor_count = 0;
  bool                     active = false;

  // health tracking
  int  consecutive_failures = 0;
  bool suspended = false;
  int  suspended_at_cycle = 0;
};

class UnitreeHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UnitreeHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  void init_cmd_buffers();
  void set_kp_kd(double kp_out, double kd_out);

  std::vector<MotorPort> ports_;

  // 12-joint state
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // 12-joint commands
  std::vector<double> hw_commands_pos_;
  std::vector<double> hw_commands_vel_;
  std::vector<double> hw_commands_eff_;

  // motor-level params
  double gear_ratio_ = 1.0;
  int    motor_mode_ = 0;
  double kp_config_  = 0.4;
  double kd_config_  = 0.01;
  int    baudrate_   = 4000000;
  int    timeout_us_ = 20000;

  bool   initialized_ = false;
  std::atomic<int> cycle_count_{0};
  std::mutex cmd_mutex_;

  static constexpr int SUSPEND_AFTER_FAILURES = 10;
  static constexpr int PROBE_INTERVAL_CYCLES  = 100;  // ~2s @200Hz
};

}  // namespace unitree_hardware_interface

#endif  // UNITREE_HARDWARE_INTERFACE__UNITREE_HARDWARE_INTERFACE_HPP
