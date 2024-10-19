#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "dynamixel_sdk/dynamixel_sdk.h"
#include <cmath>

#define DEVICE_NAME "/dev/ttyCM"
#define PROTOCOL_VERSION                2.0
#define BAUDRATE                        1000000

// Control table address
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_VELOCITY         128
#define ADDR_X_PRESENT_POSITION         132

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_GOAL_POSITION             4
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_VELOCITY          4
#define LEN_X_PRESENT_POSITION          4

// Dynamixel IDs
#define DXL_F_LEFT_ID                   1       // ID of front left motor
#define DXL_F_RIGHT_ID                  2       // ID of front right motor
#define DXL_R_LEFT_ID                   3       // ID of middle left motor
#define DXL_R_RIGHT_ID                  4       // ID of middle right motor

#define DXL_GIMBAL1_ID                  10
#define DXL_GIMBAL2_ID                  11
#define DXL_GIMBAL3_ID                  12
#define DXL_GIMBAL4_ID                  13

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0

class MotorCommand {
public:
    MotorCommand();
    ~MotorCommand();
    bool init();
    void close();
    bool setTorque(bool enable);
    bool writeYawPosition(const int64_t value[4]);
    bool writeGimbalPosition(const int64_t value[4]);
private:
    uint32_t baudrate_;
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    dynamixel::GroupSyncWrite *groupSyncWritePosition_;
};

#endif
