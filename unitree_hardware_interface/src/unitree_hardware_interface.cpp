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
    ports_.push_back(std::move(port));

    RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
                "Configured port %s → motors %d–%d",
                ports_.back().path.c_str(),
                ports_.back().base,
                ports_.back().base + MOTORS_PER_PORT - 1);
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

  // Initialize cmd buffers — kp=kd=0 so motors don't jump on first sendRecv
  init_cmd_buffers();

  // Capture initial positions with zero gains
  for (auto & port : ports_) {
    if (!port.active) continue;
    try {
      port.serial->sendRecv(port.cmds, port.data);
      for (int i = 0; i < MOTORS_PER_PORT; i++) {
        int gi = port.base + i;
        if (port.data[i].correct) {
          hw_positions_[gi]    = port.data[i].q  / gear_ratio_;
          hw_velocities_[gi]   = port.data[i].dq / gear_ratio_;
          hw_efforts_[gi]      = port.data[i].tau * gear_ratio_;
          hw_commands_pos_[gi] = hw_positions_[gi];
        }
      }
    } catch (...) { /* keep zero defaults */ }
  }

  // Now set actual kp/kd so motors hold position
  set_kp_kd(kp_config_, kd_config_);

  // Fill cmd.q with captured positions — first read() must not drive motors to 0
  for (auto & port : ports_) {
    if (!port.active) continue;
    for (int i = 0; i < MOTORS_PER_PORT; i++) {
      int gi = port.base + i;
      port.cmds[i].q = hw_commands_pos_[gi] * gear_ratio_;
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

  // Stop all motors
  for (auto & port : ports_) {
    if (!port.active) continue;
    for (int i = 0; i < MOTORS_PER_PORT; i++) {
      port.cmds[i].q   = 0.0;
      port.cmds[i].dq  = 0.0;
      port.cmds[i].tau = 0.0;
      port.cmds[i].kp  = 0.0;
      port.cmds[i].kd  = 0.0;
    }
    try { port.serial->sendRecv(port.cmds, port.data); }
    catch (...) {}
  }

  for (auto & port : ports_) {
    port.serial.reset();
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

    // Probe suspended ports periodically
    if (port.suspended) {
      if (cycle_count_ - port.suspended_at_cycle < PROBE_INTERVAL_CYCLES)
        continue;
      port.suspended = false;
    }

    futures.push_back(std::async(std::launch::async, [&port, this]() {
      bool port_ok = false;
      try {
        bool ok = port.serial->sendRecv(port.cmds, port.data);
        if (ok) {
          int valid = 0;
          for (int i = 0; i < MOTORS_PER_PORT; i++) {
            if (!port.data[i].correct) continue;
            valid++;
            int gi = port.base + i;
            hw_positions_[gi]  = port.data[i].q  / gear_ratio_;
            hw_velocities_[gi] = port.data[i].dq / gear_ratio_;
            hw_efforts_[gi]    = port.data[i].tau * gear_ratio_;
          }
          if (valid > 0) port_ok = true;
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

  double kp_r = kp_config_ / (gear_ratio_ * gear_ratio_);
  double kd_r = kd_config_ / (gear_ratio_ * gear_ratio_);

  for (auto & port : ports_) {
    if (!port.active || port.suspended) continue;

    for (int i = 0; i < MOTORS_PER_PORT; i++) {
      int gi = port.base + i;

      port.cmds[i].motorType = MotorType::GO_M8010_6;
      port.cmds[i].id        = static_cast<unsigned short>(i);
      port.cmds[i].mode      = static_cast<unsigned short>(motor_mode_);
      port.cmds[i].q         = hw_commands_pos_[gi] * gear_ratio_;
      port.cmds[i].dq        = hw_commands_vel_[gi] * gear_ratio_;
      port.cmds[i].tau       = hw_commands_eff_[gi] / gear_ratio_;
      port.cmds[i].kp        = static_cast<float>(kp_r);
      port.cmds[i].kd        = static_cast<float>(kd_r);
    }
  }

  return hardware_interface::return_type::OK;
}

// ─── Helpers ─────────────────────────────────────────────────

void UnitreeHardwareInterface::init_cmd_buffers()
{
  for (auto & port : ports_) {
    for (int i = 0; i < MOTORS_PER_PORT; i++) {
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
  double kp_r = kp_out / (gear_ratio_ * gear_ratio_);
  double kd_r = kd_out / (gear_ratio_ * gear_ratio_);

  for (auto & port : ports_) {
    for (int i = 0; i < MOTORS_PER_PORT; i++) {
      port.cmds[i].kp = static_cast<float>(kp_r);
      port.cmds[i].kd = static_cast<float>(kd_r);
    }
  }
}

}  // namespace unitree_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  unitree_hardware_interface::UnitreeHardwareInterface,
  hardware_interface::SystemInterface)
