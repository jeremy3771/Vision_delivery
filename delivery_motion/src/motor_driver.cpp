#include "delivery_motion/motor_driver.hpp"

MotorCommand::MotorCommand() {
    init();
}
MotorCommand::~MotorCommand() {
    close();
}
bool MotorCommand::init() {
    portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    if (portHandler_->openPort() == false) {
        return false;
    }
    if (portHandler_->setBaudRate(baudrate_) == false) {
        return false;
    }
    setTorque(true);
    groupSyncWritePosition_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION);
    return true;
}

void MotorCommand::close() {
    setTorque(false);
    portHandler_->closePort();
}

bool MotorCommand::setTorque(bool enable) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_F_LEFT_ID, ADDR_X_TORQUE_ENABLE, enable, &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS)
        return false;
    else if(dxl_error != 0)
        return false;

    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_F_RIGHT_ID, ADDR_X_TORQUE_ENABLE, enable, &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS)
        return false;
    else if(dxl_error != 0)
        return false;

    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_R_LEFT_ID, ADDR_X_TORQUE_ENABLE, enable, &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS)
        return false;
    else if(dxl_error != 0)
        return false;

    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_R_RIGHT_ID, ADDR_X_TORQUE_ENABLE, enable, &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS)
        return false;
    else if(dxl_error != 0)
        return false;

    return true;
}

bool MotorCommand::writePosition(const int64_t value[4]) {
    bool dxl_addparam_result;
    int8_t dxl_comm_result;
    uint8_t data_byte[4] = {0, };

    data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value[0]));
    data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value[0]));
    data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value[0]));
    data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value[0]));
    dxl_addparam_result = groupSyncWritePosition_->addParam(DXL_F_LEFT_ID, (uint8_t*)&data_byte);
    if (dxl_addparam_result != true)
        return false;

    data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value[1]));
    data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value[1]));
    data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value[1]));
    data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value[1]));
    dxl_addparam_result = groupSyncWritePosition_->addParam(DXL_F_RIGHT_ID, (uint8_t*)&data_byte);
    if (dxl_addparam_result != true)
        return false;

    data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value[2]));
    data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value[2]));
    data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value[2]));
    data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value[2]));
    dxl_addparam_result = groupSyncWritePosition_->addParam(DXL_R_LEFT_ID, (uint8_t*)&data_byte);
    if (dxl_addparam_result != true)
        return false;

    data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value[3]));
    data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value[3]));
    data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value[3]));
    data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value[3]));
    dxl_addparam_result = groupSyncWritePosition_->addParam(DXL_R_RIGHT_ID, (uint8_t*)&data_byte);
    if (dxl_addparam_result != true)
        return false;

    dxl_comm_result = groupSyncWritePosition_->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        return false;

    groupSyncWritePosition_->clearParam();
    return true;
}
