#ifndef VESCMOTORCONTROLLER_H
#define VESCMOTORCONTROLLER_H

#include <QObject>
#include <QSharedPointer>
#include "motorcontroller.h"
#include "servocontroller.h"
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QByteArray>
#include "ext/vesc/vescpacket.h"
#include "ext/vesc/datatypes.h"

class VESCMotorController : public MotorController
{
    Q_OBJECT
public:
    VESCMotorController();

    bool connectSerial(const QSerialPortInfo &serialPortInfo);
    bool isSerialConnected();

    virtual void pollFirmwareVersion();
    virtual void requestRPM(int32_t rpm);

    QSharedPointer<ServoController> getServoController();

private:
    // internal class to avoid mutli-inheritance from QObject
    class VESCServoController : public ServoController {
    public:
        VESCServoController(VESC::Packet* packet) {mVESCPacket = packet;};
        virtual void requestSteering(float steering);
    private:
        VESC::Packet* mVESCPacket;
    };
    QSharedPointer<VESCServoController> mVESCServoController;

    QSerialPort mSerialPort;

    const int heartbeatPeriod_ms = 300;
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
