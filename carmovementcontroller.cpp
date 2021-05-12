#include "carmovementcontroller.h"
#include <QDebug>

CarMovementController::CarMovementController(QSharedPointer<CarState> vehicleState): MovementController(vehicleState)
{

}

void CarMovementController::setDesiredSteering(double desiredSteering)
{
    MovementController::setDesiredSteering(desiredSteering);
    if (mServoController)
        mServoController->requestSteering(desiredSteering);

    // update vehicleState in any case (we do not expect feedback from servo)
    getVehicleState().dynamicCast<CarState>()->setSteering(desiredSteering);
}

void CarMovementController::setDesiredSpeed(double desiredSpeed)
{
    MovementController::setDesiredSpeed(desiredSpeed);
    if (mMotorController)
        mMotorController->requestRPM(desiredSpeed*getSpeedToRPMFactor());
    else {
        qDebug() << "WARNING: CarMovementController has no MotorController connection. Simulating movement."; // TODO: create explicitly simulated controller
        getVehicleState().dynamicCast<CarState>()->setSpeed(desiredSpeed);
    }
}

void CarMovementController::setMotorController(const QSharedPointer<MotorController> motorController)
{
    mMotorController = motorController;
    connect(mMotorController.get(), &MotorController::statusValuesReceived, this, &CarMovementController::updateVehicleState);
}

void CarMovementController::setServoController(const QSharedPointer<ServoController> servoController)
{
    mServoController = servoController;
}

double CarMovementController::getSpeedToRPMFactor() const
{
    return mSpeedToRPMFactor;
}

void CarMovementController::setSpeedToRPMFactor(double speedToRPMFactor)
{
    mSpeedToRPMFactor = speedToRPMFactor;
}

void CarMovementController::updateVehicleState(double rpm, int tachometer, double voltageInput, double temperature, int errorID)
{
    Q_UNUSED(tachometer) // TODO: use for odometry
    Q_UNUSED(voltageInput)
    Q_UNUSED(temperature)
    Q_UNUSED(errorID)
    getVehicleState().dynamicCast<CarState>()->setSpeed(rpm/getSpeedToRPMFactor());
}
