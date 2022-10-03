/*
 *     Copyright 2021 Marvin Damschen   marvin.damschen@ri.se
 *               2021 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#include <QDebug>
#include <QThread>
#include <QTimer>

#include "canopenmovementcontroller.h"

CANopenMovementController::CANopenMovementController(QSharedPointer<VehicleState> vehicleState): MovementController(vehicleState)
{
    //    // --- CANopen device is created in its own thread ---
    mCanopenThread.reset(new QThread);
    mCANopenControllerInterface.reset(new CANopenControllerInterface());
    mCANopenControllerInterface->moveToThread(mCanopenThread.get());
    QObject::connect(mCanopenThread.get(), &QThread::started, mCANopenControllerInterface.get(), &CANopenControllerInterface::startDevice);
    QObject::connect(mCANopenControllerInterface.get(), &CANopenControllerInterface::finished, mCanopenThread.get(), &QThread::quit);
    QObject::connect(mCANopenControllerInterface.get(), &CANopenControllerInterface::activateSimulation, [&](){
        qDebug() << "WARNING: CANopenMovementController could not connect to CAN bus, simulating movement.";
        mSimulateMovement = true;
    });
    mCanopenThread->start();

    // --- Set up communication between the threads ---
    QObject::connect(this, &CANopenMovementController::sendCommandSpeed, mCANopenControllerInterface.get(), &CANopenControllerInterface::commandSpeedReceived, Qt::QueuedConnection);
    QObject::connect(this, &CANopenMovementController::sendCommandSteeringCurvature, mCANopenControllerInterface.get(), &CANopenControllerInterface::commandSteeringReceived, Qt::QueuedConnection);
    QObject::connect(this, &CANopenMovementController::sendCommandAttributes, mCANopenControllerInterface.get(), &CANopenControllerInterface::commandAttributesReceived, Qt::QueuedConnection);
    QObject::connect(this, &CANopenMovementController::sendActualStatus, mCANopenControllerInterface.get(), &CANopenControllerInterface::actualStatusReceived, Qt::QueuedConnection);
    QObject::connect(this, &CANopenMovementController::sendGNSSDataToCAN, mCANopenControllerInterface.get(), &CANopenControllerInterface::GNSSDataToCANReceived, Qt::QueuedConnection);
    QObject::connect(this, &CANopenMovementController::txDistOfRouteLeft, mCANopenControllerInterface.get(), &CANopenControllerInterface::rxDistOfRouteLeft, Qt::QueuedConnection);

    QObject::connect(mCANopenControllerInterface.get(), &CANopenControllerInterface::sendActualSpeed, this, &CANopenMovementController::actualSpeedReceived);
    QObject::connect(mCANopenControllerInterface.get(), &CANopenControllerInterface::sendActualSteeringCurvature, this, &CANopenMovementController::actualSteeringCurvatureReceived);
    QObject::connect(mCANopenControllerInterface.get(), &CANopenControllerInterface::sendCommandStatus, this, &CANopenMovementController::commandStatusReceived);
    QObject::connect(mCANopenControllerInterface.get(), &CANopenControllerInterface::sendBatterySOC, this, &CANopenMovementController::batterySOCReceived);
    QObject::connect(mCANopenControllerInterface.get(), &CANopenControllerInterface::sendBatteryVoltage, this, &CANopenMovementController::batteryVoltageReceived);


}

void CANopenMovementController::setDesiredSteeringCurvature(double desiredSteeringCurvature)
{
    MovementController::setDesiredSteeringCurvature(desiredSteeringCurvature);

    if (!mSimulateMovement)
        emit sendCommandSteeringCurvature(desiredSteeringCurvature);
}

void CANopenMovementController::setDesiredSpeed(double desiredSpeed)
{
    MovementController::setDesiredSpeed(desiredSpeed);

    if (!mSimulateMovement)
        emit sendCommandSpeed(desiredSpeed);
    else
        getVehicleState()->setSpeed(desiredSpeed);
}

void CANopenMovementController::setDesiredSteering(double desiredSteering)
{
    MovementController::setDesiredSteering(desiredSteering);

    if (!mSimulateMovement)
        emit sendCommandSteeringCurvature(desiredSteering / (getVehicleState()->getWidth() / 2.0)); // Note: setDesiredSteering(..) not supported by current protocol
    else
        getVehicleState()->setSteering(desiredSteering);
}

void CANopenMovementController::setDesiredAttributes(quint32 desiredAttributes)
{
    MovementController::setDesiredAttributes(desiredAttributes);

    if (!mSimulateMovement)
        emit sendCommandAttributes(desiredAttributes);
}

void CANopenMovementController::actualSpeedReceived(double speed) {
    getVehicleState()->setSpeed(speed);

    static double lastSpeed = speed;
    static QTime lastTimeCalled = QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc());
    QTime thisTimeCalled = QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc());
    int dt_ms = lastTimeCalled.msecsTo(thisTimeCalled);
    double drivenDistance = ((lastSpeed + speed) / 2.0) * dt_ms / 1000.0;

    getVehicleState()->updateOdomPositionAndYaw(drivenDistance);
    emit updatedOdomPositionAndYaw(getVehicleState(), drivenDistance);

    lastSpeed = speed;
    lastTimeCalled = thisTimeCalled;
}

void CANopenMovementController::actualSteeringCurvatureReceived(double steeringCurvature) {
    getVehicleState()->setSteering(getVehicleState()->steeringCurvatureToSteering(steeringCurvature));
}

void CANopenMovementController::commandStatusReceived(quint8 status) {
    static quint8 lastStatus = 0;

    mCANOpenAutopilotControlState.emergencyStop = !((status >> 0) & 1);
    mCANOpenAutopilotControlState.autoPilot = (status >> 1) & 1;
    mCANOpenAutopilotControlState.followMe = (status >> 2) & 1;
    mCANOpenAutopilotControlState.pause = (status >> 3) & 1;
    mCANOpenAutopilotControlState.resume = (status >> 4) & 1;
    mCANOpenAutopilotControlState.routeID = (status >> 5);

    if (lastStatus != status)
        emit CANOpenAutopilotControlStateChanged(mCANOpenAutopilotControlState);

    lastStatus = status;
    emit sendActualStatus(lastStatus);
}

void CANopenMovementController::batterySOCReceived(double batterysoc) {
    mBatterySOC = batterysoc;
}

void CANopenMovementController::batteryVoltageReceived(double batteryvoltage) {
    mBatteryVoltage = batteryvoltage;
}

bool CANopenMovementController::isMovementSimulated() const
{
    return mSimulateMovement;
}

void CANopenMovementController::rxNavPvt(const ubx_nav_pvt &pvt) {
    QVariant gnssData;
    gnssData.setValue(pvt);
    emit sendGNSSDataToCAN(gnssData);
}

void CANopenMovementController::rxDistOfRouteLeft(double dist) {
    emit txDistOfRouteLeft(dist);
}
