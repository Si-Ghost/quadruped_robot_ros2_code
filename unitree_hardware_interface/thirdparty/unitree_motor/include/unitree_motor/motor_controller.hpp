#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <functional>

#include "protocol.hpp"

namespace unitree_motor {

// ── Serial port abstraction (platform-specific) ─────────────────────

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& port, int baudrate, double timeout_sec);
    void close();
    bool is_open() const;

    void write(const uint8_t* data, size_t len);
    size_t read(uint8_t* buf, size_t len);
    void flush_input();
    void flush_output();
    void drain();

private:
    int _fd = -1;
};

// ── MotorController (single RS485 bus) ──────────────────────────────

class MotorController {
public:
    MotorController(const std::string& port = "/dev/ttyUSB0",
                    int baudrate = 4'000'000,
                    double timeout = 0.01);
    ~MotorController();

    MotorController(const MotorController&) = delete;
    MotorController& operator=(const MotorController&) = delete;

    bool open();
    void close();
    bool is_open() const;

    /// Send a command and receive the motor's response (thread-safe).
    MotorState send_command(const MotorCommand& cmd);

    /// Send raw packet, return raw response bytes.
    std::vector<uint8_t> send_raw(const std::vector<uint8_t>& packet);

    // Convenience methods
    MotorState stop(uint8_t motor_id = BROADCAST_ID);
    MotorState set_torque(uint8_t motor_id, double torque);
    MotorState set_position(uint8_t motor_id, double position,
                            double kp = 1.0, double kd = 0.1,
                            double speed_limit = 30.0, double torque_limit = 10.0);
    MotorState set_speed(uint8_t motor_id, double speed);

    const std::string& port() const { return _port; }

private:
    std::string  _port;
    int          _baudrate;
    double       _timeout;
    SerialPort   _serial;
    std::mutex   _mutex;
};

// ── MotorGroup (multiple motors on one bus) ─────────────────────────

class MotorGroup {
public:
    MotorGroup(MotorController& controller, std::vector<uint8_t> motor_ids);

    const std::unordered_map<uint8_t, MotorState>& states() const { return _states; }

    std::vector<MotorState> command(const std::vector<MotorCommand>& commands);
    std::vector<MotorState> read_all();
    void stop_all();

private:
    MotorController& _ctrl;
    std::vector<uint8_t> _ids;
    std::unordered_map<uint8_t, MotorState> _states;
};

// ── MotorSystem (multiple buses) ────────────────────────────────────

class MotorSystem {
public:
    /// port_motor_map: port path -> list of motor IDs on that bus
    MotorSystem(const std::unordered_map<std::string, std::vector<uint8_t>>& port_motor_map,
                int baudrate = 4'000'000,
                double timeout = 0.01);
    ~MotorSystem();

    MotorSystem(const MotorSystem&) = delete;
    MotorSystem& operator=(const MotorSystem&) = delete;

    bool open_all();
    void close_all();

    std::vector<uint8_t> motor_ids() const;
    std::vector<std::string> ports() const;

    MotorController* get_controller(uint8_t motor_id);

    MotorState send_command(const MotorCommand& cmd);
    std::vector<MotorState> send_commands(const std::vector<MotorCommand>& commands);
    std::unordered_map<uint8_t, MotorState> read_all();

    void stop_all();
    void stop_motor(uint8_t motor_id);

private:
    std::unordered_map<std::string, std::unique_ptr<MotorController>> _controllers;
    std::unordered_map<uint8_t, std::string> _motor_to_port;
    std::unordered_map<std::string, std::unique_ptr<MotorGroup>> _groups;
    int    _baudrate;
    double _timeout;
};

} // namespace unitree_motor
