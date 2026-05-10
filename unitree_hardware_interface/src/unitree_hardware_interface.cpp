#include "unitree_hardware_interface/unitree_hardware_interface.hpp"
#include <algorithm>
#include <sstream>

namespace unitree_hardware_interface
{

// ─── Lifecycle ───────────────────────────────────────────────

hardware_interface::CallbackReturn
UnitreeHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse hardware parameters from URDF
  auto & hp = info_.hardware_parameters;//读取URDF储存的硬件字典

  if (hp.count("kp"))  kp_config_  = std::stod(hp.at("kp"));
  if (hp.count("kd"))  kd_config_  = std::stod(hp.at("kd"));
  if (hp.count("baudrate"))   baudrate_   = std::stoi(hp.at("baudrate"));
  if (hp.count("timeout_us")) timeout_us_ = std::stoi(hp.at("timeout_us"));

  // Port configuration: "port_0", "port_1", ... each "path:base"
  for (int i = 0; i < PORTS; i++) {//确认端口号和结构体
    MotorPort port;
    port.base = i * MOTORS_PER_PORT;
    port.path = "/dev/ttyUSB" + std::to_string(i);

    std::string key = "port_" + std::to_string(i);
    if (hp.count(key)) {
      auto val = hp.at(key);
      auto colon = val.find(':');
      if (colon != std::string::npos) {
        port.path = val.substr(0, colon);
        port.base = std::stoi(val.substr(colon + 1));
      }
    }

    port.cmds.resize(MOTORS_PER_PORT);
    port.data.resize(MOTORS_PER_PORT);

    // motor_count: how many motors belong to this port (min of slots and total remaining)
    int remaining = TOTAL_MOTORS - port.base;
    port.motor_count = std::min(MOTORS_PER_PORT, remaining);

    ports_.push_back(std::move(port));

    RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
                "Configured port %s → motors %d–%d (%d motors)",
                ports_.back().path.c_str(),
                ports_.back().base,
                ports_.back().base + ports_.back().motor_count - 1,
                ports_.back().motor_count);
  }

  // 12 joints × 3 state interfaces
  hw_positions_.resize(TOTAL_MOTORS, 0.0);
  hw_velocities_.resize(TOTAL_MOTORS, 0.0);
  hw_efforts_.resize(TOTAL_MOTORS, 0.0);

  // 12 joints × 3 command interfaces (pos + vel + eff)
  hw_commands_pos_.resize(TOTAL_MOTORS, 0.0);
  hw_commands_vel_.resize(TOTAL_MOTORS, 0.0);
  hw_commands_eff_.resize(TOTAL_MOTORS, 0.0);

  gear_ratio_ = queryGearRatio(MotorType::GO_M8010_6);
  motor_mode_ = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);

  RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
              "on_init OK — %zu motors, gear_ratio=%.3f, kp=%.3f, kd=%.3f",
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
    try {
      port.serial = std::make_unique<SerialPort>(
        port.path, 16, baudrate_, timeout_us_,
        BlockYN::NO, bytesize_t::eightbits,
        parity_t::parity_none, stopbits_t::stopbits_one,
        flowcontrol_t::flowcontrol_none);
      port.active = true;
      RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
                  "  %s opened OK", port.path.c_str());
    } catch (const std::exception & e) {
      port.active = false;
      RCLCPP_WARN(rclcpp::get_logger("UnitreeHardwareInterface"),
                  "  %s FAILED: %s", port.path.c_str(), e.what());
    }
  }

  long active = std::count_if(ports_.begin(), ports_.end(),
                              [](auto &p) { return p.active; });
  if (active == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("UnitreeHardwareInterface"),
                 "No serial ports available");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize cmd buffers — kp=kd=0, position=0.
  // No serial I/O here.  Initial positions are captured in the first
  // few cycles of read() which runs in the RT (SCHED_FIFO) thread.
  init_cmd_buffers();

  positions_captured_ = false;
  initialized_ = true;
  RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
              "on_activate complete — %ld/%d ports active", active, PORTS);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
UnitreeHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "on_deactivate");

  // Stop all motors
  for (auto & port : ports_) {
    if (!port.active) continue;
    for (int i = 0; i < port.motor_count; i++) {
      port.cmds[i].q   = 0.0;
      port.cmds[i].dq  = 0.0;
      port.cmds[i].tau = 0.0;
      port.cmds[i].kp  = 0.0;
      port.cmds[i].kd  = 0.0;
    }
    for (int i = 0; i < port.motor_count; i++) {
      try { port.serial->sendRecv(&port.cmds[i], &port.data[i]); }
      catch (...) {}
    }
  }

  for (auto & port : ports_) {
    port.serial.reset();
    port.active = false;
  }

  initialized_ = false;
  positions_captured_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─── Interface export ───────────────────────────────────────

std::vector<hardware_interface::StateInterface>
UnitreeHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  for (int i = 0; i < TOTAL_MOTORS; i++) {
    std::string j = info_.joints[i].name;
    ifaces.emplace_back(j, hardware_interface::HW_IF_POSITION,  &hw_positions_[i]);
    ifaces.emplace_back(j, hardware_interface::HW_IF_VELOCITY,  &hw_velocities_[i]);
    ifaces.emplace_back(j, hardware_interface::HW_IF_EFFORT,    &hw_efforts_[i]);
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

  std::vector<std::future<void>> futures;

  for (auto & port : ports_) {
    if (!port.active) continue;

    if (port.suspended) {
      if (cycle_count_ - port.suspended_at_cycle < PROBE_INTERVAL_CYCLES)
        continue;
      port.suspended = false;
    }

    futures.push_back(std::async(std::launch::async, [&port, this]() {
      bool port_ok = false;
      try {
        int valid = 0;
        bool all_ok = true;
        for (int i = 0; i < port.motor_count; i++) {
          bool ok = port.serial->sendRecv(&port.cmds[i], &port.data[i]);
          if (ok && port.data[i].correct) {
            valid++;
            int gi = port.base + i;
            hw_positions_[gi]  = port.data[i].q  / gear_ratio_;
            hw_velocities_[gi] = port.data[i].dq / gear_ratio_;
            hw_efforts_[gi]    = port.data[i].tau * gear_ratio_;
            // Capture initial position (RT thread — reliable timing)
            if (!positions_captured_) {
              hw_commands_pos_[gi] = hw_positions_[gi];
            }
          } else {
            all_ok = false;
          }
        }
        if (valid > 0) port_ok = true;

        // Once all motors on this port respond in one cycle, startup is done
        if (!positions_captured_ && all_ok && valid == port.motor_count) {
          positions_captured_ = true;
          RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
                      "%s initial positions captured at cycle %d",
                      port.path.c_str(), cycle_count_.load());
        }
      } catch (...) { /* port_ok stays false */ }

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
    }));
  }

  for (auto & f : futures) f.wait();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
UnitreeHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!initialized_) return hardware_interface::return_type::OK;

  std::lock_guard<std::mutex> lock(cmd_mutex_);

  // kp/kd 是电机原始坐标系的值，不做 gear_ratio 换算（与 Python 版一致）
  for (auto & port : ports_) {
    if (!port.active || port.suspended) continue;

    for (int i = 0; i < port.motor_count; i++) {
      int gi = port.base + i;

      port.cmds[i].motorType = MotorType::GO_M8010_6;
      port.cmds[i].id        = static_cast<unsigned short>(i);
      port.cmds[i].mode      = static_cast<unsigned short>(motor_mode_);

      port.cmds[i].dq  = hw_commands_vel_[gi] * gear_ratio_;
      port.cmds[i].tau = hw_commands_eff_[gi] / gear_ratio_;

      if (positions_captured_) {
        double cmd_pos = hw_commands_pos_[gi];
        if (cycle_count_ <= STARTUP_GUARD_CYCLES &&
            cmd_pos == 0.0 && hw_positions_[gi] != 0.0) {
          cmd_pos = hw_positions_[gi];
        }
        port.cmds[i].q  = cmd_pos * gear_ratio_;
        port.cmds[i].kp = static_cast<float>(kp_config_);
        port.cmds[i].kd = static_cast<float>(kd_config_);
      } else {
        // Startup: keep kp=kd=0, don't drive motor until position is captured
        port.cmds[i].q  = hw_commands_pos_[gi] * gear_ratio_;
        port.cmds[i].kp = 0.0;
        port.cmds[i].kd = 0.0;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

// ─── Helpers ─────────────────────────────────────────────────

void UnitreeHardwareInterface::init_cmd_buffers()
{
  for (auto & port : ports_) {
    for (int i = 0; i < port.motor_count; i++) {
      port.cmds[i].motorType = MotorType::GO_M8010_6;
      port.data[i].motorType = MotorType::GO_M8010_6;
      port.cmds[i].id        = static_cast<unsigned short>(i);
      port.cmds[i].mode      = static_cast<unsigned short>(motor_mode_);
      port.cmds[i].q         = 0.0f;
      port.cmds[i].dq        = 0.0f;
      port.cmds[i].tau       = 0.0f;
      port.cmds[i].kp        = 0.0f;
      port.cmds[i].kd        = 0.0f;
    }
  }
}

void UnitreeHardwareInterface::set_kp_kd(double kp_out, double kd_out)
{
  for (auto & port : ports_) {
    for (int i = 0; i < port.motor_count; i++) {
      port.cmds[i].kp = static_cast<float>(kp_out);
      port.cmds[i].kd = static_cast<float>(kd_out);
    }
  }
}

}  // namespace unitree_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  unitree_hardware_interface::UnitreeHardwareInterface,
  hardware_interface::SystemInterface)
