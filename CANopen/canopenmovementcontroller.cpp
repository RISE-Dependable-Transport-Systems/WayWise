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
    //QObject::connect(worker, SIGNAL(error(QString)), this, SLOT(errorString(QString)));
    QObject::connect(mCanopenThread.get(), &QThread::started, mCANopenControllerInterface.get(), &CANopenControllerInterface::startDevice);
    QObject::connect(mCANopenControllerInterface.get(), &CANopenControllerInterface::finished, mCanopenThread.get(), &QThread::quit);
    QObject::connect(mCANopenControllerInterface.get(), &CANopenControllerInterface::activateSimulation, [&](){
        qDebug() << "WARNING: CANopenMovementController could not connect to CAN bus, simulating movement.";
        mSimulateMovement = true;
    });
    mCanopenThread->start();

    // --- Set up communication between the threads ---
    QObject::connect(this, &CANopenMovementController::sendCommandSpeed, mCANopenControllerInterface.get(), &CANopenControllerInterface::commandSpeedReceived);
    QObject::connect(this, &CANopenMovementController::sendCommandSteeringCurvature, mCANopenControllerInterface.get(), &CANopenControllerInterface::commandSteeringReceived);
    QObject::connect(this, &CANopenMovementController::sendCommandAttributes, mCANopenControllerInterface.get(), &CANopenControllerInterface::commandAttributesReceived);
    QObject::connect(this, &CANopenMovementController::sendActualStatus, mCANopenControllerInterface.get(), &CANopenControllerInterface::actualStatusReceived);

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
        ; // CANopenMovementController::setDesiredSteering(double desiredSteering) not supported by current protocol
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

    emit sendActualStatus(status); // TODO: ack back to CAN

    if (lastStatus != status)
        emit CANOpenAutopilotControlStateChanged(mCANOpenAutopilotControlState);

    lastStatus = status;
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
