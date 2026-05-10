#include "unitree_motor/motor_controller.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <cstring>
#include <cerrno>
#include <iostream>

// Linux-specific: struct termios2 for arbitrary baud rates.
// glibc <termios.h> does not expose termios2; <asm/termbits.h> is
// missing on some architectures (e.g. ARM64).  Define it ourselves.
#ifndef BOTHER
struct termios2 {
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_cflag;
    tcflag_t c_lflag;
    cc_t     c_line;
    cc_t     c_cc[19];
    speed_t  c_ispeed;
    speed_t  c_ospeed;
};
#define BOTHER  0x1000
#define TCGETS2 _IOR('T', 0x2A, struct termios2)
#define TCSETS2 _IOW('T', 0x2B, struct termios2)
#endif

namespace unitree_motor {

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& port, int baudrate, double timeout_sec) {
    if (_fd >= 0) return true;

    _fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_fd < 0) {
        std::cerr << "Cannot open " << port << ": " << strerror(errno) << std::endl;
        return false;
    }

    // ── arbitrary baud rate via termios2 ──────────────────────
    struct termios2 tty;
    std::memset(&tty, 0, sizeof(tty));

    // fetch current settings, modify, apply
    if (ioctl(_fd, TCGETS2, &tty) < 0) {
        std::cerr << "TCGETS2 failed: " << strerror(errno) << std::endl;
        ::close(_fd);
        _fd = -1;
        return false;
    }

    tty.c_cflag = CS8 | CREAD | CLOCAL | BOTHER;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_ispeed = static_cast<speed_t>(baudrate);
    tty.c_ospeed = static_cast<speed_t>(baudrate);

    // blocking read with inter-byte timeout
    tty.c_cc[VMIN]  = static_cast<cc_t>(RECV_PACKET_SIZE);  // wait for full 16-byte response
    tty.c_cc[VTIME] = static_cast<cc_t>(std::max(1.0, timeout_sec * 10.0));

    if (ioctl(_fd, TCSETS2, &tty) < 0) {
        std::cerr << "TCSETS2 failed: " << strerror(errno) << std::endl;
        ::close(_fd);
        _fd = -1;
        return false;
    }

    tcflush(_fd, TCIOFLUSH);
    return true;
}

void SerialPort::close() {
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
    }
}

bool SerialPort::is_open() const {
    return _fd >= 0;
}

void SerialPort::write(const uint8_t* data, size_t len) {
    if (_fd < 0) return;
    ssize_t written = ::write(_fd, data, len);
    if (written < 0)
        std::cerr << "Serial write error: " << strerror(errno) << std::endl;
}

size_t SerialPort::read(uint8_t* buf, size_t len) {
    if (_fd < 0) return 0;

    // poll with a short timeout for RT safety
    // (VTIME handles the normal wait; poll is a fail-safe against infinite block)
    struct pollfd pfd;
    pfd.fd = _fd;
    pfd.events = POLLIN;
    int ret = poll(&pfd, 1, 2);  // 2 ms hard cap (enough for 4Mbps response)
    if (ret <= 0)
        return 0;

    ssize_t n = ::read(_fd, buf, len);
    if (n < 0) {
        if (errno != EAGAIN)
            std::cerr << "Serial read error: " << strerror(errno) << std::endl;
        return 0;
    }
    return static_cast<size_t>(n);
}

void SerialPort::flush_input() {
    if (_fd >= 0) tcflush(_fd, TCIFLUSH);
}

void SerialPort::flush_output() {
    if (_fd >= 0) tcflush(_fd, TCOFLUSH);
}

void SerialPort::drain() {
    if (_fd >= 0) tcdrain(_fd);
}

} // namespace unitree_motor
