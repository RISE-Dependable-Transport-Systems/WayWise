#include "vescmotorcontroller.h"
#include "ext/vesc/datatypes.h"
#include "vbytearray.h"
#include <QDebug>

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
    mHeartbeatTimer.start(heartbeatPeriod_ms);

    // periodically poll VESC state
    connect(&mPollValuesTimer, &QTimer::timeout, this, [this](){
        VByteArray vb;
        vb.vbAppendInt8(VESC::COMM_GET_VALUES_SELECTIVE);
        vb.vbAppendUint32(SELECT_VALUES_MASK);
        mVESCPacket.sendPacket(vb);});
    mPollValuesTimer.start(pollValuesPeriod_ms);
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

void VESCMotorController::VESCServoController::requestSteering(float steering) // TODO: smoothen servo input
{
    VByteArray vb;
    vb.vbAppendInt8(VESC::COMM_SET_SERVO_POS);
    vb.vbAppendDouble16((steering + 1.0) / 2.0, 1000.0); // input steering in [-1.0:1.0], but VESC steering is [0.0:1.0]
    mVESCPacket->sendPacket(vb);
}

QSharedPointer<ServoController> VESCMotorController::getServoController()
{
    return mVESCServoController;
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
    case VESC::COMM_GET_VALUES:
    case VESC::COMM_GET_VALUES_SELECTIVE: {
        VESC::MC_VALUES values;

        uint32_t mask = 0xFFFFFFFF;
        if (id == VESC::COMM_GET_VALUES_SELECTIVE) {
            mask = vb.vbPopFrontUint32();
        }

        if (mask & (uint32_t(1) << 0)) {
            values.temp_mos = vb.vbPopFrontDouble16(1e1);
        }
        if (mask & (uint32_t(1) << 1)) {
            values.temp_motor = vb.vbPopFrontDouble16(1e1);
        }
        if (mask & (uint32_t(1) << 2)) {
            values.current_motor = vb.vbPopFrontDouble32(1e2);
        }
        if (mask & (uint32_t(1) << 3)) {
            values.current_in = vb.vbPopFrontDouble32(1e2);
        }
        if (mask & (uint32_t(1) << 4)) {
            values.id = vb.vbPopFrontDouble32(1e2);
        }
        if (mask & (uint32_t(1) << 5)) {
            values.iq = vb.vbPopFrontDouble32(1e2);
        }
        if (mask & (uint32_t(1) << 6)) {
            values.duty_now = vb.vbPopFrontDouble16(1e3);
        }
        if (mask & (uint32_t(1) << 7)) {
            values.rpm = vb.vbPopFrontDouble32(1e0);
        }
        if (mask & (uint32_t(1) << 8)) {
            values.v_in = vb.vbPopFrontDouble16(1e1);
        }
        if (mask & (uint32_t(1) << 9)) {
            values.amp_hours = vb.vbPopFrontDouble32(1e4);
        }
        if (mask & (uint32_t(1) << 10)) {
            values.amp_hours_charged = vb.vbPopFrontDouble32(1e4);
        }
        if (mask & (uint32_t(1) << 11)) {
            values.watt_hours = vb.vbPopFrontDouble32(1e4);
        }
        if (mask & (uint32_t(1) << 12)) {
            values.watt_hours_charged = vb.vbPopFrontDouble32(1e4);
        }
        if (mask & (uint32_t(1) << 13)) {
            values.tachometer = vb.vbPopFrontInt32();
        }
        if (mask & (uint32_t(1) << 14)) {
            values.tachometer_abs = vb.vbPopFrontInt32();
        }
        if (mask & (uint32_t(1) << 15)) {
            values.fault_code = VESC::mc_fault_code(vb.vbPopFrontInt8());
            values.fault_str = VESCFaultToStr(values.fault_code);
        }

        if (vb.size() >= 4) {
            if (mask & (uint32_t(1) << 16)) {
                values.position = vb.vbPopFrontDouble32(1e6);
            }
        } else {
            values.position = -1.0;
        }

        if (vb.size() >= 1) {
            if (mask & (uint32_t(1) << 17)) {
                values.vesc_id = vb.vbPopFrontUint8();
            }
        } else {
            values.vesc_id = 255;
        }

        if (vb.size() >= 6) {
            if (mask & (uint32_t(1) << 18)) {
                values.temp_mos_1 = vb.vbPopFrontDouble16(1e1);
                values.temp_mos_2 = vb.vbPopFrontDouble16(1e1);
                values.temp_mos_3 = vb.vbPopFrontDouble16(1e1);
            }
        }

        if (vb.size() >= 8) {
            if (mask & (uint32_t(1) << 19)) {
                values.vd = vb.vbPopFrontDouble32(1e3);
            }
            if (mask & (uint32_t(1) << 20)) {
                values.vq = vb.vbPopFrontDouble32(1e3);
            }
        }
//        qDebug() << "Temp.:" << values.temp_mos << "RPM:" << values.rpm << "VIN:" << values.v_in << "Tacho.:" << values.tachometer << "Error:" << values.fault_str;
        emit statusValuesReceived(values.rpm, values.tachometer, values.v_in, values.temp_mos, values.fault_code);
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
