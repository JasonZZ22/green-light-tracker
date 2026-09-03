#include "serial_comm.h"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cerrno>
#include <iostream>

// MAVLink 头文件（c_library_v2 已在 CMake include path 中）
#include "common/mavlink.h"

bool SerialComm::open(const char* device, int baud) {
    fd_ = ::open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
        std::cerr << "[Serial] 无法打开 " << device << ": " << strerror(errno) << std::endl;
        return false;
    }

    // 设为阻塞模式
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NDELAY);

    struct termios tty{};
    tcgetattr(fd_, &tty);

    // 波特率
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // 8N1, 无流控
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    tcsetattr(fd_, TCSANOW, &tty);
    tcflush(fd_, TCIOFLUSH);

    std::cout << "[Serial] 已打开 " << device << " @ " << baud << " baud" << std::endl;
    return true;
}

void SerialComm::close() {
    if (fd_ >= 0) {
        std::cout << "[Serial] 关闭串口" << std::endl;
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialComm::sendServoCmd(uint16_t ch1, uint16_t ch2,
                               uint16_t ch3, uint16_t ch4) {
    if (fd_ < 0) return false;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // target_system=1, target_component=1 (飞控 ID)
    mavlink_msg_rc_channels_override_pack(
        255,     // 本机 system ID
        1,       // 本机 component ID
        &msg,
        1,       // 目标 system ID (飞控)
        1,       // 目标 component ID
        ch1, ch2, ch3, ch4,   // 通道 1-4
        0, 0, 0, 0,           // 通道 5-8
        0, 0, 0, 0, 0, 0, 0, 0,   // 通道 9-16
        0, 0                    // 通道 17-18
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ssize_t written = ::write(fd_, buf, len);

    return written == static_cast<ssize_t>(len);
}
