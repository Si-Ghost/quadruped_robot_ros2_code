#include "unitree_hardware_interface/unitree_hardware_interface.hpp"
#include <algorithm>

namespace unitree_hardware_interface
{

using unitree_motor::MotorCommand;
using unitree_motor::MotorState;
using unitree_motor::MODE_FOC;
using unitree_motor::BROADCAST_ID;

// ─── Lifecycle ───────────────────────────────────────────────

hardware_interface::CallbackReturn
UnitreeHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto & hp = info_.hardware_parameters;

  if (hp.count("kp"))  kp_config_  = std::stod(hp.at("kp"));
  if (hp.count("kd"))  kd_config_  = std::stod(hp.at("kd"));
  if (hp.count("baudrate"))   baudrate_   = std::stoi(hp.at("baudrate"));
  timeout_sec_ = 0.02;
  if (hp.count("timeout_us")) timeout_sec_ = std::stoi(hp.at("timeout_us")) / 1e6;

  // gear_ratio: GO-M8010-6 = 6.33
  gear_ratio_ = 6.33;

  // Parse port config → each port gets a list of motor local-IDs
  for (int i = 0; i < PORTS; i++) {
    MotorPort port;
    port.path = "/dev/ttyUSB" + std::to_string(i);
    port.base = i * 3;  // max 3 motors per port

    std::string key = "port_" + std::to_string(i);
    if (hp.count(key)) {
      auto val = hp.at(key);
      auto colon = val.find(':');
      if (colon != std::string::npos) {
        port.path = val.substr(0, colon);
        port.base = std::stoi(val.substr(colon + 1));
      }
    }

    // Populate motor IDs: local ID on this RS485 bus (0, 1, 2, ...)
    int n_motors = std::min(3, TOTAL_MOTORS - port.base);
    for (int j = 0; j < n_motors; j++) {
      port.motor_ids.push_back(static_cast<uint8_t>(j));
    }

    port.cmds.resize(port.motor_ids.size());
    port.states.resize(port.motor_ids.size());
    ports_.push_back(std::move(port));

    RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
                "Configured port %s → %d motors (global %d–%d)",
                ports_.back().path.c_str(), n_motors,
                ports_.back().base,
                ports_.back().base + n_motors - 1);
  }

  hw_positions_.resize(TOTAL_MOTORS, 0.0);
  hw_velocities_.resize(TOTAL_MOTORS, 0.0);
  hw_efforts_.resize(TOTAL_MOTORS, 0.0);

  hw_commands_pos_.resize(TOTAL_MOTORS, 0.0);
  hw_commands_vel_.resize(TOTAL_MOTORS, 0.0);
  hw_commands_eff_.resize(TOTAL_MOTORS, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
              "on_init OK — %d motors, gear_ratio=%.3f, kp=%.3f, kd=%.3f",
              TOTAL_MOTORS, gear_ratio_, kp_config_, kd_config_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
UnitreeHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "on_configure");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
UnitreeHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "on_activate — opening ports");

  for (auto & port : ports_) {
    port.controller = std::make_unique<unitree_motor::MotorController>(
        port.path, baudrate_, timeout_sec_);
    if (port.controller->open()) {
      port.active = true;
      RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
                  "  %s opened OK", port.path.c_str());
    } else {
      port.active = false;
      RCLCPP_WARN(rclcpp::get_logger("UnitreeHardwareInterface"),
                  "  %s FAILED", port.path.c_str());
    }
  }

  long active = std::count_if(ports_.begin(), ports_.end(),
                              [](auto &p) { return p.active; });
  if (active == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("UnitreeHardwareInterface"),
                 "No serial ports available");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize command buffers (kp=kd=0, position=0)
  for (auto & port : ports_) {
    if (!port.active) continue;
    for (size_t i = 0; i < port.motor_ids.size(); i++) {
      port.cmds[i].motor_id = port.motor_ids[i];
      port.cmds[i].mode     = MODE_FOC;
      port.cmds[i].position = 0.0;
      port.cmds[i].speed    = 0.0;
      port.cmds[i].torque   = 0.0;
      port.cmds[i].kp       = 0.0;
      port.cmds[i].kd       = 0.0;
    }
  }

  // Read initial positions (per-motor send_command, retry 3x)
  read_initial_positions();

  // Now set kp/kd to configured gains and fill position = current
  for (auto & port : ports_) {
    if (!port.active) continue;
    for (size_t i = 0; i < port.motor_ids.size(); i++) {
      int gi = port.base + static_cast<int>(i);
      port.cmds[i].kp       = static_cast<float>(kp_config_);
      port.cmds[i].kd       = static_cast<float>(kd_config_);
      port.cmds[i].position = hw_commands_pos_[gi] * gear_ratio_;
    }
    // Send initial hold command to motors before control loop starts
    for (size_t i = 0; i < port.motor_ids.size(); i++) {
      try {
        port.controller->send_command(port.cmds[i]);
      } catch (...) {}
    }
  }

  initialized_ = true;
  RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
              "on_activate complete — %ld/%d ports active", active, PORTS);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
UnitreeHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "on_deactivate");

  for (auto & port : ports_) {
    if (!port.active || !port.controller) continue;
    try {
      port.controller->stop();
    } catch (...) {}
    port.controller->close();
    port.active = false;
  }

  initialized_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─── Interface export ───────────────────────────────────────

std::vector<hardware_interface::StateInterface>
UnitreeHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  for (int i = 0; i < TOTAL_MOTORS; i++) {
    std::string j = info_.joints[i].name;
    ifaces.emplace_back(j, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    ifaces.emplace_back(j, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    ifaces.emplace_back(j, hardware_interface::HW_IF_EFFORT,   &hw_efforts_[i]);
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface>
UnitreeHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (int i = 0; i < TOTAL_MOTORS; i++) {
    std::string j = info_.joints[i].name;
    ifaces.emplace_back(j, hardware_interface::HW_IF_POSITION, &hw_commands_pos_[i]);
    ifaces.emplace_back(j, hardware_interface::HW_IF_VELOCITY, &hw_commands_vel_[i]);
    ifaces.emplace_back(j, hardware_interface::HW_IF_EFFORT,   &hw_commands_eff_[i]);
  }
  return ifaces;
}

// ─── Main I/O ────────────────────────────────────────────────

hardware_interface::return_type
UnitreeHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!initialized_) return hardware_interface::return_type::OK;

  cycle_count_++;

  for (auto & port : ports_) {
    if (!port.active || !port.controller) continue;

    if (port.suspended) {
      if (cycle_count_ - port.suspended_at_cycle < PROBE_INTERVAL_CYCLES)
        continue;
      port.suspended = false;
    }

    bool port_ok = false;
    int valid = 0;

    for (size_t i = 0; i < port.motor_ids.size(); i++) {
      try {
        auto state = port.controller->send_command(port.cmds[i]);
        if (state.valid) {
          valid++;
          int gi = port.base + static_cast<int>(i);
          hw_positions_[gi]  = state.position / gear_ratio_;
          hw_velocities_[gi] = state.speed    / gear_ratio_;
          hw_efforts_[gi]    = state.torque   * gear_ratio_;
          port.states[i] = state;
        }
      } catch (...) { /* skip this motor */ }
    }

    if (valid > 0) port_ok = true;

    if (port_ok) {
      port.consecutive_failures = 0;
    } else {
      port.consecutive_failures++;
      if (port.consecutive_failures >= SUSPEND_AFTER_FAILURES) {
        if (!port.suspended) {
          RCLCPP_WARN(rclcpp::get_logger("UnitreeHardwareInterface"),
                      "Suspending %s after %d failures",
                      port.path.c_str(), port.consecutive_failures);
        }
        port.suspended = true;
        port.suspended_at_cycle = cycle_count_;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
UnitreeHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!initialized_) return hardware_interface::return_type::OK;

  std::lock_guard<std::mutex> lock(cmd_mutex_);

  for (auto & port : ports_) {
    if (!port.active || port.suspended) continue;

    for (size_t i = 0; i < port.motor_ids.size(); i++) {
      int gi = port.base + static_cast<int>(i);

      double cmd_pos = hw_commands_pos_[gi];
      if (cycle_count_ <= STARTUP_GUARD_CYCLES &&
          cmd_pos == 0.0 && hw_positions_[gi] != 0.0) {
        cmd_pos = hw_positions_[gi];
      }

      port.cmds[i].motor_id = port.motor_ids[i];
      port.cmds[i].mode     = MODE_FOC;
      port.cmds[i].position = cmd_pos * gear_ratio_;
      port.cmds[i].speed    = hw_commands_vel_[gi] * gear_ratio_;
      port.cmds[i].torque   = hw_commands_eff_[gi] / gear_ratio_;
      port.cmds[i].kp       = static_cast<float>(kp_config_);
      port.cmds[i].kd       = static_cast<float>(kd_config_);
    }
  }

  return hardware_interface::return_type::OK;
}

// ─── Helpers ─────────────────────────────────────────────────

void UnitreeHardwareInterface::read_initial_positions()
{
  for (auto & port : ports_) {
    if (!port.active || !port.controller) continue;

    for (size_t i = 0; i < port.motor_ids.size(); i++) {
      int gi = port.base + static_cast<int>(i);
      bool ok = false;

      for (int attempt = 0; attempt < 3 && !ok; attempt++) {
        MotorCommand cmd;
        cmd.motor_id = port.motor_ids[i];
        cmd.mode     = MODE_FOC;
        cmd.position = 0.0;
        cmd.speed    = 0.0;
        cmd.torque   = 0.0;
        cmd.kp       = 0.0;   // zero gains for safe initial read
        cmd.kd       = 0.0;

        try {
          auto state = port.controller->send_command(cmd);
          if (state.valid) {
            hw_positions_[gi]    = state.position / gear_ratio_;
            hw_velocities_[gi]   = state.speed    / gear_ratio_;
            hw_efforts_[gi]      = state.torque   * gear_ratio_;
            hw_commands_pos_[gi] = hw_positions_[gi];
            ok = true;
          }
        } catch (...) {}

        if (!ok) {
          RCLCPP_WARN(rclcpp::get_logger("UnitreeHardwareInterface"),
                      "%s motor %u init read attempt %d failed, retrying...",
                      port.path.c_str(), port.motor_ids[i], attempt + 1);
        }
      }

      if (!ok) {
        RCLCPP_ERROR(rclcpp::get_logger("UnitreeHardwareInterface"),
                     "%s motor %u failed after 3 attempts", port.path.c_str(), port.motor_ids[i]);
      }
    }
  }
}

}  // namespace unitree_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  unitree_hardware_interface::UnitreeHardwareInterface,
  hardware_interface::SystemInterface)
