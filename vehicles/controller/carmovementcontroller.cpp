/*
 *     Copyright 2021 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "carmovementcontroller.h"
#include <QDebug>

CarMovementController::CarMovementController(QSharedPointer<CarState> vehicleState, bool autoActuateMotorAndServo): MovementController(vehicleState)
{
    mCarState = getVehicleState().dynamicCast<CarState>();
    mAutoActuateMotorAndServo = autoActuateMotorAndServo;
}

void CarMovementController::setDesiredSteering(double desiredSteering)
{
    if (abs(desiredSteering) > 1.0)
        desiredSteering = (desiredSteering > 0) ? 1.0 : -1.0;

    MovementController::setDesiredSteering(desiredSteering);
    // update vehicleState in any case (we do not expect feedback from servo)
    mCarState->setSteering(desiredSteering);

    if (mAutoActuateMotorAndServo) {
        actuateSteeringServo(desiredSteering);
    }
}

void CarMovementController::setDesiredSpeed(double desiredSpeed)
{
    MovementController::setDesiredSpeed(desiredSpeed);

    if (mAutoActuateMotorAndServo) {
        actuateDriveMotor(desiredSpeed*getSpeedToRPMFactor());
    }
}

void CarMovementController::setMotorController(const QSharedPointer<MotorController> motorController)
{
    mMotorController = motorController;
    connect(mMotorController.get(), &MotorController::gotStatusValues, this, &CarMovementController::updateVehicleState);
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

void CarMovementController::updateVehicleState(double rpm, int tachometer, int tachometer_abs, double voltageInput, double temperature, int errorID)
{
    Q_UNUSED(tachometer_abs)
    Q_UNUSED(voltageInput)
    Q_UNUSED(temperature)
    Q_UNUSED(errorID)

    static int previousTachometer = tachometer;
    QSharedPointer<CarState> carState = getVehicleState().dynamicCast<CarState>();
    double currentSpeed = rpm/getSpeedToRPMFactor();
    double drivenDistance = (tachometer - previousTachometer)/getSpeedToRPMFactor() * 60.0;

    xyz_t currentVelocity = carState->getVelocity();
    currentVelocity.x = currentSpeed;
    carState->setVelocity(currentVelocity);
    carState->updateOdomPositionAndYaw(drivenDistance);

    previousTachometer = tachometer;
    emit updatedOdomPositionAndYaw(carState, drivenDistance);
}

void CarMovementController::simulationStep(double dt_ms) {
    double drivenDistance = getDesiredSpeed() * dt_ms / 1000;

    xyz_t currentVelocity = mCarState->getVelocity();
    currentVelocity.x = getDesiredSpeed();
    mCarState->setVelocity(currentVelocity);

    mCarState->updateOdomPositionAndYaw(drivenDistance);
    emit updatedOdomPositionAndYaw(mCarState, drivenDistance);
}

void CarMovementController::actuateDriveMotor(int32_t rpm)
{
    if (mMotorController) {
        mMotorController->requestRPM(rpm);
    }
}

void CarMovementController::actuateSteeringServo(float steering)
{
    if (mServoController) {
        mServoController->requestSteering(steering);
    }
}
