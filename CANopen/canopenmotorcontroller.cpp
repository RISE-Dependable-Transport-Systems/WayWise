#include <QDebug>
#include <QThread>
#include <QTimer>

#include "canopenmotorcontroller.h"
#include "canopencontrollerinterface.h"

CANopenMotorController::CANopenMotorController() {
//    // --- CANopen device is created in its own thread ---
    QThread* mCanopenThread = new QThread;
    CANopenMotorControllerInterface* mCANopenMotorControllerInterface = new CANopenMotorControllerInterface();
    mCANopenMotorControllerInterface->moveToThread(mCanopenThread);
    //QObject::connect(worker, SIGNAL(error(QString)), this, SLOT(errorString(QString)));
    QObject::connect(mCanopenThread, SIGNAL(started()), mCANopenMotorControllerInterface, SLOT(startDevice()));
    QObject::connect(mCANopenMotorControllerInterface, SIGNAL(finished()), mCanopenThread, SLOT(quit()));
    QObject::connect(mCANopenMotorControllerInterface, SIGNAL(finished()), mCANopenMotorControllerInterface, SLOT(deleteLater()));
    QObject::connect(mCanopenThread, SIGNAL(finished()), mCanopenThread, SLOT(deleteLater()));
    mCanopenThread->start();

    // --- Set up communication between the threads ---
    QObject::connect(this, SIGNAL(sendCommandSpeed(qint8)), mCANopenMotorControllerInterface, SLOT(commandSpeedReceived(qint8)));
    QObject::connect(this, SIGNAL(sendCommandSteering(qint16)), mCANopenMotorControllerInterface, SLOT(commandSteeringReceived(qint16)));
    QObject::connect(mCANopenMotorControllerInterface, SIGNAL(sendActualSpeed(qint8)), this, SLOT(actualSpeedReceived(qint8)));
    QObject::connect(mCANopenMotorControllerInterface, SIGNAL(sendActualSteering(qint16)), this, SLOT(actualSteeringReceived(qint16)));
    QObject::connect(mCANopenMotorControllerInterface, SIGNAL(sendStatus(quint8)), this, SLOT(statusReceived(quint8)));
    QObject::connect(mCANopenMotorControllerInterface, SIGNAL(sendBatterySOC(quint8)), this, SLOT(batterySOCReceived(quint8)));
    QObject::connect(mCANopenMotorControllerInterface, SIGNAL(sendBatteryVoltage(quint16)), this, SLOT(batteryVoltageReceived(quint16)));

}

CANopenMotorController::~CANopenMotorController() {

}

void CANopenMotorController::pollFirmwareVersion() {
    // TODO
}

void CANopenMotorController::requestRPM(int32_t rpm) {
    // TODO
}

void CANopenMotorController::setCommandSpeed(qint8 speed) {
    emit sendCommandSpeed(speed);
}

void CANopenMotorController::setCommandSteering(qint16 steering) {
    emit sendCommandSteering(steering);
}

qint16 CANopenMotorController::getActualSteering() {
    return mActualSteering;
}

qint8 CANopenMotorController::getActualSpeed() {
    return mActualSpeed;
}

quint8 CANopenMotorController::getStatus() {
    return mStatus;
}

quint8 CANopenMotorController::getBatterySOC() {
    return mBatterySOC;
}

quint16 CANopenMotorController::getBatteryVoltage() {
    return mBatteryVoltage;
}

void CANopenMotorController::actualSpeedReceived(qint8 speed) {
    mActualSpeed = speed;
}

void CANopenMotorController::actualSteeringReceived(qint16 steering) {
    mActualSteering = steering;
}

void CANopenMotorController::statusReceived(quint8 status) {
    mStatus = status;
}

void CANopenMotorController::batterySOCReceived(quint8 batterysoc) {
    mBatterySOC = batterysoc;
}

void CANopenMotorController::batteryVoltageReceived(quint16 batteryvoltage) {
    mBatteryVoltage = batteryvoltage;
}
