#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <atomic>

/**
 * @brief 串口 + MAVLink 通信模块
 *
 * 通过串口发送 RC_CHANNELS_OVERRIDE 消息给下位机（飞控），
 * 下位机负责将 PWM 信号转发给舵机执行。
 *
 * 协议: MAVLink v2, 115200 8N1
 * 消息: RC_CHANNELS_OVERRIDE (#70)
 */
class SerialComm {
public:
    SerialComm() = default;
    ~SerialComm() { close(); }

    SerialComm(const SerialComm&) = delete;
    SerialComm& operator=(const SerialComm&) = delete;

    /**
     * @brief 打开串口
     * @param device  串口设备路径，如 "/dev/ttyS1"
     * @param baud    波特率，默认 115200
     * @return 是否成功
     */
    bool open(const char* device, int baud = 115200);

    /** 关闭串口 */
    void close();

    /** 是否已打开 */
    bool isOpen() const { return fd_ >= 0; }

    /**
     * @brief 发送舵机控制指令 (RC Override)
     * @param ch1~ch4  4 个通道的 PWM 脉宽 (us)，范围 1000-2000
     *                 1500 = 中位, 1000 = 最小值, 2000 = 最大值
     */
    bool sendServoCmd(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4);

    /** 便捷: 角度 (0-180°) → PWM 脉宽 (1000-2000us) */
    static uint16_t angleToPwm(float degrees) {
        if (degrees < 0) degrees = 0;
        if (degrees > 180) degrees = 180;
        return static_cast<uint16_t>(1000 + (degrees / 180.0f) * 1000);
    }

private:
    int fd_ = -1;
};
