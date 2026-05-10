#ifdef _WIN32
#include "unitree_motor/motor_controller.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <iostream>

namespace unitree_motor {

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& port, int baudrate, double timeout_sec) {
    if (_fd >= 0) return true;

    std::string dev = "\\\\.\\" + port;  // COM1..COM9 need \\.\ prefix
    HANDLE h = CreateFileA(
        dev.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0, nullptr, OPEN_EXISTING, 0, nullptr
    );
    if (h == INVALID_HANDLE_VALUE) {
        std::cerr << "Cannot open " << port << std::endl;
        return false;
    }

    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(h, &dcb);
    dcb.BaudRate = baudrate;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    SetCommState(h, &dcb);

    COMMTIMEOUTS to = {};
    to.ReadIntervalTimeout         = 0;
    to.ReadTotalTimeoutMultiplier  = 0;
    to.ReadTotalTimeoutConstant    = static_cast<DWORD>(timeout_sec * 1000.0);
    to.WriteTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant   = 100;
    SetCommTimeouts(h, &to);

    _fd = reinterpret_cast<intptr_t>(h);
    return true;
}

void SerialPort::close() {
    if (_fd >= 0) {
        CloseHandle(reinterpret_cast<HANDLE>(_fd));
        _fd = -1;
    }
}

bool SerialPort::is_open() const {
    return _fd >= 0;
}

void SerialPort::write(const uint8_t* data, size_t len) {
    if (_fd < 0) return;
    DWORD written = 0;
    WriteFile(reinterpret_cast<HANDLE>(_fd), data, static_cast<DWORD>(len), &written, nullptr);
}

size_t SerialPort::read(uint8_t* buf, size_t len) {
    if (_fd < 0) return 0;
    DWORD n = 0;
    ReadFile(reinterpret_cast<HANDLE>(_fd), buf, static_cast<DWORD>(len), &n, nullptr);
    return n;
}

void SerialPort::flush_input() {
    if (_fd >= 0)
        PurgeComm(reinterpret_cast<HANDLE>(_fd), PURGE_RXCLEAR);
}

void SerialPort::flush_output() {
    if (_fd >= 0)
        PurgeComm(reinterpret_cast<HANDLE>(_fd), PURGE_TXCLEAR);
}

void SerialPort::drain() {
    if (_fd >= 0)
        FlushFileBuffers(reinterpret_cast<HANDLE>(_fd));
}

} // namespace unitree_motor
#endif // _WIN32
