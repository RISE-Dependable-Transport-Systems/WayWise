#include "vescmotorcontroller.h"
#include "ext/vesc/datatypes.h"
#include "vbytearray.h"
#include <QDebug>

VESCMotorController::VESCMotorController(QObject *parent) : QObject(parent)
{
    connect(&mSerialPort, &QSerialPort::readyRead, this, [this](){
        while (mSerialPort.bytesAvailable() > 0)
            mVESCPacket.processData(mSerialPort.readAll());
    });

    connect(&mVESCPacket, &VESC::Packet::packetReceived, this, &VESCMotorController::processVESCPacket);
    connect(&mVESCPacket, &VESC::Packet::dataToSend, this, [this](QByteArray &data){
        if (mSerialPort.isOpen()) {
            mSerialPort.write(data);
        }});
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

void VESCMotorController::pollFirmwareVersion()
{
    VByteArray vb;
    vb.vbAppendInt8(VESC::COMM_FW_VERSION);
    mVESCPacket.sendPacket(vb);
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
        qDebug().nospace() << "VESC firmware " << mVescFirmwareInfo.major << "." << mVescFirmwareInfo.minor << " on hardware version " << mVescFirmwareInfo.hw;
        emit firmwareVersionReceived(QPair<int,int>(mVescFirmwareInfo.major, mVescFirmwareInfo.minor));
    } break;
    default:
        qDebug() << "WARNING: unhandles VESC command with id" << id;
    }
}
