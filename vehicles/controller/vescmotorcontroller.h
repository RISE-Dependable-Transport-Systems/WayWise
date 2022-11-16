/*
 *     Copyright 2021 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Implementation of the abstract MotorController for "VESC" motor controllers.
 * VESC contains an IMU and can control servos. Respective IMUOrientationUpdater and ServoController can be obtained using getters.
 */

#ifndef VESCMOTORCONTROLLER_H
#define VESCMOTORCONTROLLER_H

#include <QObject>
#include <QSharedPointer>
#include "motorcontroller.h"
#include "servocontroller.h"
#include "sensors/imu/imuorientationupdater.h"
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QByteArray>
#include "external/vesc/vescpacket.h"
#include "external/vesc/datatypes.h"

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
    QSharedPointer<IMUOrientationUpdater> getIMUOrientationUpdater(QSharedPointer<VehicleState> vehicleState);

    int getPollValuesPeriod() const;
    void setPollValuesPeriod(int milliseconds);

private:
    // internal classes to avoid mutli-inheritance from QObject
    class VESCServoController : public ServoController {
    public:
        VESCServoController(VESC::Packet* packet) {mVESCPacket = packet;};
        virtual void requestSteering(float steering);
    private:
        VESC::Packet* mVESCPacket;
    };
    QSharedPointer<VESCServoController> mVESCServoController;

    class VESCOrientationUpdater : public IMUOrientationUpdater {
    public:
        VESCOrientationUpdater(QSharedPointer<VehicleState> vehicleState) : IMUOrientationUpdater(vehicleState) {}
        virtual bool setUpdateIntervall(int) override {
            return false; // TODO
        }
    private:
        void useIMUDataFromVESC(double roll, double pitch, double yaw) {
            QSharedPointer<VehicleState> vehicleState = getVehicleState();
            PosPoint currIMUPos = vehicleState->getPosition(PosType::IMU);

            currIMUPos.setRollPitchYaw(roll, pitch, coordinateTransforms::yawNEDtoENU(yaw));
            currIMUPos.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
            vehicleState->setPosition(currIMUPos);

            emit updatedIMUOrientation(vehicleState);
        };

        friend class VESCMotorController;
    };

    QSerialPort mSerialPort;

    const int heartbeatPeriod_ms = 300;
    QTimer mHeartbeatTimer;

    // Only request selected values
    const unsigned TMOS_MASK = ((uint32_t)1 << 0);
    const unsigned CURRENT_MOTOR_MASK = ((uint32_t)1 << 2);
    const unsigned CURRENT_IN_MASK = ((uint32_t)1 << 3);
    const unsigned RPM_MASK = ((uint32_t)1 << 7);
    const unsigned VIN_MASK = ((uint32_t)1 << 8);
    const unsigned TACHO_MASK = ((uint32_t)1 << 13);
    const unsigned TACHO_ABS_MASK = ((uint32_t)1 << 14);
    const unsigned FAULT_MASK = ((uint32_t)1 << 15);
    const unsigned SELECT_VALUES_MASK = TMOS_MASK | CURRENT_MOTOR_MASK | CURRENT_IN_MASK | RPM_MASK | VIN_MASK | TACHO_MASK | TACHO_ABS_MASK | FAULT_MASK;

    const unsigned ROLL_MASK = ((uint32_t)1 << 0);
    const unsigned PITCH_MASK = ((uint32_t)1 << 1);
    const unsigned YAW_MASK = ((uint32_t)1 << 2);
    const unsigned SELECT_IMU_DATA_MASK = ROLL_MASK | PITCH_MASK | YAW_MASK;

    int pollValuesPeriod_ms = 20;
    QTimer mPollValuesTimer;

    const int checkCurrentPeriod_ms = 100;
    QTimer mCheckCurrentTimer;
    int mLastRPMrequest = 0;
    const int MAX_RPM_CONSIDERED_STOP = 500; // motor will not move when RPM lower than this are requested

    VESC::Packet mVESCPacket;
    VESC::FW_RX_PARAMS mVescFirmwareInfo;

    bool mEnableIMUOrientationUpdate = false;
    void setEnableIMUOrientationUpdate(bool enabled);
    QSharedPointer<VESCOrientationUpdater> mVESCOrientationUpdater;

    void processVESCPacket(QByteArray &data);
    QString VESCFaultToStr(VESC::mc_fault_code fault);
};

#endif // VESCMOTORCONTROLLER_H
