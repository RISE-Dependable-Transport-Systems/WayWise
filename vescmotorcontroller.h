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

    void requestRPM(int32_t rpm);
    void requestSteering(float steering);

signals:
    void firmwareVersionReceived(QPair<int,int>); // Major and minor firmware version
    void statusValuesReceived(double rpm, int tachometer, double voltageInput, double temperature, int errorID);

private:
    QSerialPort mSerialPort;

    const int heartbeatPeriod_ms = 100;
    QTimer mHeartbeatTimer;

    // Only request selected values
    const unsigned TMOS_MASK = ((uint32_t)1 << 0);
    const unsigned RPM_MASK = ((uint32_t)1 << 7);
    const unsigned VIN_MASK = ((uint32_t)1 << 8);
    const unsigned TACHO_MASK = ((uint32_t)1 << 13);
    const unsigned FAULT_MASK = ((uint32_t)1 << 15);
    const unsigned SELECT_VALUES_MASK = TMOS_MASK | RPM_MASK | VIN_MASK | TACHO_MASK | FAULT_MASK;
    const int pollValuesPeriod_ms = 20;
    QTimer mPollValuesTimer;

    VESC::Packet mVESCPacket;
    VESC::FW_RX_PARAMS mVescFirmwareInfo;

    void processVESCPacket(QByteArray &data);
    QString VESCFaultToStr(VESC::mc_fault_code fault);
};

#endif // VESCMOTORCONTROLLER_H
