#include "unitree_motor/motor_controller.hpp"

#include <cstring>
#include <iostream>

namespace unitree_motor {

// ══════════════════════════════════════════════════════════════════════
// MotorController
// ══════════════════════════════════════════════════════════════════════

MotorController::MotorController(const std::string& port, int baudrate, double timeout)
    : _port(port), _baudrate(baudrate), _timeout(timeout) {}

MotorController::~MotorController() {
    close();
}

bool MotorController::open() {
    if (_serial.is_open()) return true;
    if (!_serial.open(_port, _baudrate, _timeout)) {
        std::cerr << "Failed to open " << _port << std::endl;
        return false;
    }
    std::cout << "Opened " << _port << " at " << _baudrate << " baud" << std::endl;
    return true;
}

void MotorController::close() {
    if (_serial.is_open()) {
        _serial.close();
        std::cout << "Closed " << _port << std::endl;
    }
}

bool MotorController::is_open() const {
    return _serial.is_open();
}

MotorState MotorController::send_command(const MotorCommand& cmd) {
    uint8_t packet[SEND_PACKET_SIZE];
    pack_command(cmd, packet);

    std::lock_guard<std::mutex> lock(_mutex);

    if (!_serial.is_open())
        return MotorState{};

    _serial.flush_input();
    _serial.write(packet, SEND_PACKET_SIZE);
    _serial.drain();

    if (cmd.motor_id == BROADCAST_ID)
        return MotorState{};

    uint8_t response[RECV_PACKET_SIZE];
    size_t n = _serial.read(response, RECV_PACKET_SIZE);

    if (n != RECV_PACKET_SIZE) {
        std::cerr << "Short read: expected " << RECV_PACKET_SIZE
                  << ", got " << n << " bytes" << std::endl;
        return MotorState{};
    }

    return unpack_response(response);
}

std::vector<uint8_t> MotorController::send_raw(const std::vector<uint8_t>& packet) {
    std::lock_guard<std::mutex> lock(_mutex);

    if (!_serial.is_open())
        return {};

    _serial.flush_input();
    _serial.write(packet.data(), packet.size());
    _serial.drain();

    std::vector<uint8_t> response(RECV_PACKET_SIZE);
    size_t n = _serial.read(response.data(), RECV_PACKET_SIZE);
    response.resize(n);
    return response;
}

MotorState MotorController::stop(uint8_t motor_id) {
    MotorCommand cmd;
    cmd.motor_id = motor_id;
    cmd.mode = MODE_STOP;
    return send_command(cmd);
}

MotorState MotorController::set_torque(uint8_t motor_id, double torque) {
    MotorCommand cmd;
    cmd.motor_id = motor_id;
    cmd.mode = MODE_FOC;
    cmd.torque = torque;
    return send_command(cmd);
}

MotorState MotorController::set_position(uint8_t motor_id, double position,
                                          double kp, double kd,
                                          double speed_limit, double torque_limit) {
    MotorCommand cmd;
    cmd.motor_id = motor_id;
    cmd.mode     = MODE_FOC;
    cmd.position = position;
    cmd.speed    = speed_limit;
    cmd.torque   = torque_limit;
    cmd.kp       = kp;
    cmd.kd       = kd;
    return send_command(cmd);
}

MotorState MotorController::set_speed(uint8_t motor_id, double speed) {
    MotorCommand cmd;
    cmd.motor_id = motor_id;
    cmd.mode     = MODE_FOC;
    cmd.speed    = speed;
    return send_command(cmd);
}

// ══════════════════════════════════════════════════════════════════════
// MotorGroup
// ══════════════════════════════════════════════════════════════════════

MotorGroup::MotorGroup(MotorController& controller, std::vector<uint8_t> motor_ids)
    : _ctrl(controller), _ids(std::move(motor_ids)) {}

std::vector<MotorState> MotorGroup::command(const std::vector<MotorCommand>& commands) {
    std::vector<MotorState> results;
    for (const auto& cmd : commands) {
        auto state = _ctrl.send_command(cmd);
        if (state.valid)
            _states[cmd.motor_id] = state;
        results.push_back(state);
    }
    return results;
}

std::vector<MotorState> MotorGroup::read_all() {
    std::vector<MotorState> results;
    for (auto mid : _ids) {
        MotorCommand cmd;
        cmd.motor_id = mid;
        cmd.mode = MODE_FOC;
        auto state = _ctrl.send_command(cmd);
        if (state.valid)
            _states[mid] = state;
        results.push_back(state);
    }
    return results;
}

void MotorGroup::stop_all() {
    _ctrl.stop(BROADCAST_ID);
}

// ══════════════════════════════════════════════════════════════════════
// MotorSystem
// ══════════════════════════════════════════════════════════════════════

MotorSystem::MotorSystem(
    const std::unordered_map<std::string, std::vector<uint8_t>>& port_motor_map,
    int baudrate, double timeout)
    : _baudrate(baudrate), _timeout(timeout)
{
    for (const auto& [port, motor_ids] : port_motor_map) {
        auto ctrl = std::make_unique<MotorController>(port, baudrate, timeout);
        _groups[port] = std::make_unique<MotorGroup>(*ctrl, motor_ids);
        for (auto mid : motor_ids)
            _motor_to_port[mid] = port;
        _controllers[port] = std::move(ctrl);
    }
}

MotorSystem::~MotorSystem() {
    close_all();
}

bool MotorSystem::open_all() {
    bool all_ok = true;
    for (auto& [port, ctrl] : _controllers) {
        if (!ctrl->open())
            all_ok = false;
    }
    return all_ok;
}

void MotorSystem::close_all() {
    for (auto& [port, ctrl] : _controllers)
        ctrl->close();
}

std::vector<uint8_t> MotorSystem::motor_ids() const {
    std::vector<uint8_t> ids;
    ids.reserve(_motor_to_port.size());
    for (const auto& [mid, _] : _motor_to_port)
        ids.push_back(mid);
    return ids;
}

std::vector<std::string> MotorSystem::ports() const {
    std::vector<std::string> p;
    p.reserve(_controllers.size());
    for (const auto& [port, _] : _controllers)
        p.push_back(port);
    return p;
}

MotorController* MotorSystem::get_controller(uint8_t motor_id) {
    auto it = _motor_to_port.find(motor_id);
    if (it == _motor_to_port.end())
        return nullptr;
    return _controllers[it->second].get();
}

MotorState MotorSystem::send_command(const MotorCommand& cmd) {
    auto it = _motor_to_port.find(cmd.motor_id);
    if (it == _motor_to_port.end()) {
        std::cerr << "Motor " << static_cast<int>(cmd.motor_id)
                  << " not assigned to any bus" << std::endl;
        return MotorState{};
    }
    return _controllers[it->second]->send_command(cmd);
}

std::vector<MotorState> MotorSystem::send_commands(const std::vector<MotorCommand>& commands) {
    // Group commands by bus
    std::unordered_map<std::string, std::vector<MotorCommand>> by_port;
    for (const auto& cmd : commands) {
        auto it = _motor_to_port.find(cmd.motor_id);
        if (it != _motor_to_port.end())
            by_port[it->second].push_back(cmd);
    }

    std::vector<MotorState> results;
    std::mutex results_mutex;

    std::vector<std::thread> threads;
    for (auto& [port, cmds] : by_port) {
        threads.emplace_back([this, port = port, &cmds, &results, &results_mutex]() {
            for (const auto& cmd : cmds) {
                auto state = _controllers[port]->send_command(cmd);
                std::lock_guard<std::mutex> lock(results_mutex);
                results.push_back(state);
            }
        });
    }

    for (auto& t : threads)
        t.join();

    return results;
}

std::unordered_map<uint8_t, MotorState> MotorSystem::read_all() {
    std::unordered_map<uint8_t, MotorState> all_states;
    std::mutex states_mutex;

    std::vector<std::thread> threads;
    for (auto& [port, group] : _groups) {
        threads.emplace_back([this, port = port, &all_states, &states_mutex]() {
            auto it = _groups.find(port);
            if (it == _groups.end()) return;
            auto states = it->second->read_all();
            std::lock_guard<std::mutex> lock(states_mutex);
            for (const auto& s : states) {
                if (s.valid)
                    all_states[s.motor_id] = s;
            }
        });
    }

    for (auto& t : threads)
        t.join();

    return all_states;
}

void MotorSystem::stop_all() {
    for (auto& [port, ctrl] : _controllers)
        ctrl->stop(BROADCAST_ID);
}

void MotorSystem::stop_motor(uint8_t motor_id) {
    MotorCommand cmd;
    cmd.motor_id = motor_id;
    cmd.mode = MODE_STOP;
    send_command(cmd);
}

} // namespace unitree_motor
