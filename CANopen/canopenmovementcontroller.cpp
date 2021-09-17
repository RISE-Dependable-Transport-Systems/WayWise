#include <QDebug>
#include <QThread>
#include <QTimer>

#include "canopenmovementcontroller.h"
#include "canopencontrollerinterface.h"

CANopenMovementController::CANopenMovementController(QSharedPointer<DiffDriveVehicleState> vehicleState): MovementController(vehicleState)
{
    mDiffDriveVehicleState = getVehicleState().dynamicCast<DiffDriveVehicleState>();

    //    // --- CANopen device is created in its own thread ---
    QThread* mCanopenThread = new QThread;
    CANopenControllerInterface* mCANopenControllerInterface = new CANopenControllerInterface();
    mCANopenControllerInterface->moveToThread(mCanopenThread);
    //QObject::connect(worker, SIGNAL(error(QString)), this, SLOT(errorString(QString)));
    QObject::connect(mCanopenThread, SIGNAL(started()), mCANopenControllerInterface, SLOT(startDevice()));
    QObject::connect(mCANopenControllerInterface, SIGNAL(finished()), mCanopenThread, SLOT(quit()));
    QObject::connect(mCANopenControllerInterface, SIGNAL(finished()), mCANopenControllerInterface, SLOT(deleteLater()));
    QObject::connect(mCanopenThread, SIGNAL(finished()), mCanopenThread, SLOT(deleteLater()));
    mCanopenThread->start();

    // --- Set up communication between the threads ---
    QObject::connect(this, SIGNAL(sendCommandSpeed(double)), mCANopenControllerInterface, SLOT(commandSpeedReceived(double)));
    QObject::connect(this, SIGNAL(sendCommandSteering(double)), mCANopenControllerInterface, SLOT(commandSteeringReceived(double)));
    QObject::connect(this, SIGNAL(sendCommandAttributes(quint32)), mCANopenControllerInterface, SLOT(commandAttributesReceived(quint32)));
    QObject::connect(this,SIGNAL(sendActualStatus(quint8)), mCANopenControllerInterface, SLOT(actualStatusReceived(quint8)));
    QObject::connect(mCANopenControllerInterface, SIGNAL(sendActualSpeed(double)), this, SLOT(actualSpeedReceived(double)));
    QObject::connect(mCANopenControllerInterface, SIGNAL(sendActualSteering(double)), this, SLOT(actualSteeringReceived(double)));
    QObject::connect(mCANopenControllerInterface, SIGNAL(sendCommandStatus(quint8)), this, SLOT(commandStatusReceived(quint8)));
    QObject::connect(mCANopenControllerInterface, SIGNAL(sendBatterySOC(double)), this, SLOT(batterySOCReceived(double)));
    QObject::connect(mCANopenControllerInterface, SIGNAL(sendBatteryVoltage(double)), this, SLOT(batteryVoltageReceived(double)));
}

void CANopenMovementController::setDesiredSteeringCurvature(double desiredSteeringAngle)
{
    MovementController::setDesiredSteeringCurvature(desiredSteeringAngle);
    emit sendCommandSteering(desiredSteeringAngle);
}

void CANopenMovementController::setDesiredSpeed(double desiredSpeed)
{
    MovementController::setDesiredSpeed(desiredSpeed);
    mDiffDriveVehicleState->setSpeedLeft(desiredSpeed + desiredSpeed * getDesiredSteering());
    mDiffDriveVehicleState->setSpeedRight(desiredSpeed - desiredSpeed * getDesiredSteering());
    emit sendCommandSpeed(desiredSpeed);
}

void CANopenMovementController::setDesiredSteering(double desiredSteering)
{
    MovementController::setDesiredSteering(desiredSteering);
    mDiffDriveVehicleState->setSpeedLeft(getDesiredSpeed() + (getDesiredSpeed() * desiredSteering));
    mDiffDriveVehicleState->setSpeedRight(getDesiredSpeed() - (getDesiredSpeed() * desiredSteering));
}

void CANopenMovementController::setDesiredAttributes(quint32 desiredAttributes)
{
    MovementController::setDesiredAttributes(desiredAttributes);
    emit sendCommandAttributes(desiredAttributes);
}

double CANopenMovementController::getActualSteeringCurvature() {
    return mActualSteeringAngle;
}

double CANopenMovementController::getActualSpeed() {
    return mActualSpeed;
}

quint8 CANopenMovementController::getStatus() {
    return mStatus;
}

double CANopenMovementController::getBatterySOC() {
    return mBatterySOC;
}

double CANopenMovementController::getBatteryVoltage() {
    return mBatteryVoltage;
}

void CANopenMovementController::actualSpeedReceived(double speed) {
    mActualSpeed = speed;
}

void CANopenMovementController::actualSteeringReceived(double steering) {
    mActualSteeringAngle = steering;
}

void CANopenMovementController::commandStatusReceived(quint8 status) {
    mStatus = status;
    emit sendActualStatus(mStatus);
}

void CANopenMovementController::batterySOCReceived(double batterysoc) {
    mBatterySOC = batterysoc;
}

void CANopenMovementController::batteryVoltageReceived(double batteryvoltage) {
    mBatteryVoltage = batteryvoltage;
}
