#ifndef VESCMOTORCONTROLLER_H
#define VESCMOTORCONTROLLER_H

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QByteArray>
#include "ext/vesc/vescpacket.h"
#include "ext/vesc/datatypes.h"

class VESCMotorController : public QObject
{
    Q_OBJECT
public:
    explicit VESCMotorController(QObject *parent = nullptr);

    bool connectSerial(const QSerialPortInfo &serialPortInfo);
    void pollFirmwareVersion();
signals:
    void firmwareVersionReceived(QPair<int,int>); // Major and minor firmware version

private:
    QSerialPort mSerialPort;
    VESC::Packet mVESCPacket;
    VESC::FW_RX_PARAMS mVescFirmwareInfo;

    void processVESCPacket(QByteArray &data);
};

#endif // VESCMOTORCONTROLLER_H
