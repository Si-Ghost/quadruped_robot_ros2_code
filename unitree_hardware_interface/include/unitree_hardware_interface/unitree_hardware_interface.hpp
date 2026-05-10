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

#include "unitree_motor/motor_controller.hpp"

namespace unitree_hardware_interface
{

static constexpr int TOTAL_MOTORS  = 3;
static constexpr int PORTS         = 1;

struct MotorPort
{
  std::string                              path;
  int                                      base = 0;
  std::unique_ptr<unitree_motor::MotorController> controller;
  std::vector<uint8_t>                     motor_ids;   // local IDs on this bus
  std::vector<unitree_motor::MotorCommand> cmds;
  std::vector<unitree_motor::MotorState>   states;
  bool                                     active = false;

  int  consecutive_failures = 0;
  bool suspended = false;
  int  suspended_at_cycle = 0;

  int motor_count() const { return static_cast<int>(motor_ids.size()); }
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
  void read_initial_positions();

  std::vector<MotorPort> ports_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  std::vector<double> hw_commands_pos_;
  std::vector<double> hw_commands_vel_;
  std::vector<double> hw_commands_eff_;

  double gear_ratio_  = 1.0;
  int    baudrate_    = 4000000;
  double timeout_sec_ = 0.02;
  double kp_config_   = 0.4;
  double kd_config_   = 0.01;

  bool   initialized_ = false;
  std::atomic<int> cycle_count_{0};
  std::mutex cmd_mutex_;

  static constexpr int SUSPEND_AFTER_FAILURES = 10;
  static constexpr int PROBE_INTERVAL_CYCLES  = 100;
  static constexpr int STARTUP_GUARD_CYCLES   = 5;
};

}  // namespace unitree_hardware_interface

#endif  // UNITREE_HARDWARE_INTERFACE__UNITREE_HARDWARE_INTERFACE_HPP
