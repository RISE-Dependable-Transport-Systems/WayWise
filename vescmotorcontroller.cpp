#include "vescmotorcontroller.h"
#include "ext/vesc/datatypes.h"
#include "vbytearray.h"
#include <QDebug>
#include <cmath>

VESCMotorController::VESCMotorController()
{
    mVESCServoController.reset(new VESCServoController(&mVESCPacket));

    // --- Serial communication & command parsing setup
    connect(&mSerialPort, &QSerialPort::readyRead, this, [this](){
        while (mSerialPort.bytesAvailable() > 0)
            mVESCPacket.processData(mSerialPort.readAll());
    });

    connect(&mVESCPacket, &VESC::Packet::packetReceived, this, &VESCMotorController::processVESCPacket);
    connect(&mVESCPacket, &VESC::Packet::dataToSend, this, [this](QByteArray &data){
        if (mSerialPort.isOpen()) {
            mSerialPort.write(data);
        }});

    // send periodic heartbeats to prevent VESC timeout, TODO: connect to IP communication timeout?
    connect(&mHeartbeatTimer, &QTimer::timeout, this, [this](){
        VByteArray vb;
        vb.vbAppendInt8(VESC::COMM_ALIVE);
        mVESCPacket.sendPacket(vb);});

    // periodically poll VESC state and optionally IMU
    connect(&mPollValuesTimer, &QTimer::timeout, this, [this](){
        VByteArray packetData;
        packetData.vbAppendUint8(VESC::COMM_GET_VALUES_SELECTIVE);
        packetData.vbAppendUint32(SELECT_VALUES_MASK);
        mVESCPacket.sendPacket(packetData);

        if (mEnableIMUOrientationUpdate) {
            packetData.clear();
            packetData.vbAppendUint8(VESC::COMM_GET_IMU_DATA);
            packetData.vbAppendUint16(SELECT_IMU_DATA_MASK);
            mVESCPacket.sendPacket(packetData);
        }
    });
}

bool VESCMotorController::connectSerial(const QSerialPortInfo &serialPortInfo)
{
    if(mSerialPort.isOpen()) {
        mSerialPort.close();
    }

    mSerialPort.setPort(serialPortInfo);
    mSerialPort.open(QIODevice::ReadWrite);

    if(!mSerialPort.isOpen()) {
        return false;
    }

    mSerialPort.setBaudRate(115200);
    mSerialPort.setDataBits(QSerialPort::Data8);
    mSerialPort.setParity(QSerialPort::NoParity);
    mSerialPort.setStopBits(QSerialPort::OneStop);
    mSerialPort.setFlowControl(QSerialPort::NoFlowControl);

    pollFirmwareVersion();


    mPollValuesTimer.start(pollValuesPeriod_ms);
    mHeartbeatTimer.start(heartbeatPeriod_ms);

    return true;
}

bool VESCMotorController::isSerialConnected()
{
    return mSerialPort.isOpen() && mSerialPort.isWritable();
}

void VESCMotorController::pollFirmwareVersion()
{
    VByteArray vb;
    vb.vbAppendInt8(VESC::COMM_FW_VERSION);
    mVESCPacket.sendPacket(vb);
}

void VESCMotorController::requestRPM(int32_t rpm)
{
    VByteArray vb;
    vb.vbAppendInt8(VESC::COMM_SET_RPM);
    vb.vbAppendInt32(rpm);
    mVESCPacket.sendPacket(vb);
}

void VESCMotorController::setEnableIMUOrientationUpdate(bool enabled)
{
    mEnableIMUOrientationUpdate = enabled;
}

void VESCMotorController::VESCServoController::requestSteering(float steering) // TODO: smoothen servo input
{
    VByteArray vb;
    vb.vbAppendInt8(VESC::COMM_SET_SERVO_POS);
    // input steering in [-1.0:1.0], but VESC steering is [0.0:1.0], optionally invert
    steering *= (getInvertOutput() ? -1.0 : 1.0);
    vb.vbAppendDouble16((steering + 1.0) / 2.0, 1000.0);
    mVESCPacket->sendPacket(vb);
}

QSharedPointer<ServoController> VESCMotorController::getServoController()
{
    return mVESCServoController;
}

int VESCMotorController::getPollValuesPeriod() const
{
    return pollValuesPeriod_ms;
}

void VESCMotorController::setPollValuesPeriod(int milliseconds)
{
    pollValuesPeriod_ms = milliseconds;
    mPollValuesTimer.start(pollValuesPeriod_ms);
}

void VESCMotorController::processVESCPacket(QByteArray &data)
{
    VByteArray vb(data);
    VESC::COMM_PACKET_ID id = VESC::COMM_PACKET_ID(vb.vbPopFrontUint8());

    switch (id) {
    case VESC::COMM_FW_VERSION: {
        if (vb.size() >= 2) {
            mVescFirmwareInfo.major = vb.vbPopFrontInt8();
            mVescFirmwareInfo.minor = vb.vbPopFrontInt8();
            mVescFirmwareInfo.hw = vb.vbPopFrontString();
        }

        if (vb.size() >= 12) {
            mVescFirmwareInfo.uuid.append(vb.left(12));
            vb.remove(0, 12);
        }

        if (vb.size() >= 1) {
            mVescFirmwareInfo.isPaired = vb.vbPopFrontInt8();
        }

        if (vb.size() >= 1) {
            mVescFirmwareInfo.isTestFw = vb.vbPopFrontInt8();
        }

        if (vb.size() >= 1) {
            mVescFirmwareInfo.hwType = VESC::HW_TYPE(vb.vbPopFrontInt8());
        }

        if (vb.size() >= 1) {
            mVescFirmwareInfo.customConfigNum = vb.vbPopFrontInt8();
        }

        //qDebug().nospace() << "VESC firmware " << mVescFirmwareInfo.major << "." << mVescFirmwareInfo.minor << " on hardware version " << mVescFirmwareInfo.hw;

        QPair<int,int> firmwareVersionPair(mVescFirmwareInfo.major, mVescFirmwareInfo.minor);
        if (firmwareVersionPair != QPair<int,int>(5,2)) // TODO hardcoded
            qDebug().nospace() << "WARNING: VESC Firmware version " << firmwareVersionPair.first << "." << firmwareVersionPair.second << "does not match tested version.";

        emit firmwareVersionReceived(firmwareVersionPair);
    } break;

    case VESC::COMM_GET_VALUES_SELECTIVE: {
        VESC::MC_VALUES values;

        uint32_t mask = vb.vbPopFrontUint32();
        if (mask != SELECT_VALUES_MASK)
            qDebug() << "Warning: VescMotorController got COMM_GET_VALUES_SELECTIVE but mask does not match selected values.";

        values.temp_mos = vb.vbPopFrontDouble16(1e1);
        values.rpm = vb.vbPopFrontDouble32(1e0);
        values.v_in = vb.vbPopFrontDouble16(1e1);
        values.tachometer = vb.vbPopFrontInt32();
        values.tachometer_abs = vb.vbPopFrontInt32();
        values.fault_code = VESC::mc_fault_code(vb.vbPopFrontInt8());
        values.fault_str = VESCFaultToStr(values.fault_code);

//        qDebug() << "Temp.:" << values.temp_mos << "RPM:" << values.rpm << "VIN:" << values.v_in << "Tacho.:" << values.tachometer << "Tacho. Abs.:" << values.tachometer_abs << "Error:" << values.fault_str;
        emit gotStatusValues(values.rpm, values.tachometer,  values.tachometer_abs, values.v_in, values.temp_mos, values.fault_code);
    } break;

    case VESC::COMM_GET_IMU_DATA: {
        VESC::IMU_VALUES values;

        uint32_t mask = vb.vbPopFrontUint16();
        if (mask != SELECT_IMU_DATA_MASK)
            qDebug() << "Warning: VescMotorController got COMM_GET_IMU_DATA but mask does not match selected values.";

        values.roll = vb.vbPopFrontDouble32Auto();
        values.pitch = vb.vbPopFrontDouble32Auto();
        values.yaw = vb.vbPopFrontDouble32Auto();

//        qDebug() << values.roll* 180.0 / M_PI << values. pitch* 180.0 / M_PI << values.yaw* 180.0 / M_PI;
        emit gotIMUOrientation(values.roll * 180.0 / M_PI, values.pitch * 180.0 / M_PI, values.yaw * 180.0 / M_PI);
    } break;
    default:
        qDebug() << "WARNING: unhandled VESC command with id" << id;
    }
}

QString VESCMotorController::VESCFaultToStr(VESC::mc_fault_code fault)
{
    switch (fault) {
    case VESC::FAULT_CODE_NONE: return "FAULT_CODE_NONE";
    case VESC::FAULT_CODE_OVER_VOLTAGE: return "FAULT_CODE_OVER_VOLTAGE";
    case VESC::FAULT_CODE_UNDER_VOLTAGE: return "FAULT_CODE_UNDER_VOLTAGE";
    case VESC::FAULT_CODE_DRV: return "FAULT_CODE_DRV";
    case VESC::FAULT_CODE_ABS_OVER_CURRENT: return "FAULT_CODE_ABS_OVER_CURRENT";
    case VESC::FAULT_CODE_OVER_TEMP_FET: return "FAULT_CODE_OVER_TEMP_FET";
    case VESC::FAULT_CODE_OVER_TEMP_MOTOR: return "FAULT_CODE_OVER_TEMP_MOTOR";
    case VESC::FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE: return "FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE";
    case VESC::FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE: return "FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE";
    case VESC::FAULT_CODE_MCU_UNDER_VOLTAGE: return "FAULT_CODE_MCU_UNDER_VOLTAGE";
    case VESC::FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET: return "FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET";
    case VESC::FAULT_CODE_ENCODER_SPI: return "FAULT_CODE_ENCODER_SPI";
    case VESC::FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE: return "FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE";
    case VESC::FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE: return "FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE";
    case VESC::FAULT_CODE_FLASH_CORRUPTION: return "FAULT_CODE_FLASH_CORRUPTION";
    case VESC::FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1: return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1";
    case VESC::FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2: return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2";
    case VESC::FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3: return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3";
    case VESC::FAULT_CODE_UNBALANCED_CURRENTS: return "FAULT_CODE_UNBALANCED_CURRENTS";
    case VESC::FAULT_CODE_RESOLVER_LOT: return "FAULT_CODE_RESOLVER_LOT";
    case VESC::FAULT_CODE_RESOLVER_DOS: return "FAULT_CODE_RESOLVER_DOS";
    case VESC::FAULT_CODE_RESOLVER_LOS: return "FAULT_CODE_RESOLVER_LOS";
    case VESC::FAULT_CODE_FLASH_CORRUPTION_APP_CFG: return "FAULT_CODE_FLASH_CORRUPTION_APP_CFG";
    case VESC::FAULT_CODE_FLASH_CORRUPTION_MC_CFG: return "FAULT_CODE_FLASH_CORRUPTION_MC_CFG";
    case VESC::FAULT_CODE_ENCODER_NO_MAGNET: return "FAULT_CODE_ENCODER_NO_MAGNET";
    default: return "Unknown fault";
    }
}
