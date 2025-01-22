/*
 *     Copyright 2021 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "carmovementcontroller.h"
#include <QDebug>

CarMovementController::CarMovementController(QSharedPointer<CarState> vehicleState): MovementController(vehicleState)
{
    mCarState = getVehicleState().dynamicCast<CarState>();
}

void CarMovementController::setDesiredSteering(double desiredSteering)
{
    if (abs(desiredSteering) > 1.0)
        desiredSteering = (desiredSteering > 0) ? 1.0 : -1.0;

    MovementController::setDesiredSteering(desiredSteering);
    // update vehicleState in any case (we do not expect feedback from servo)
    mCarState->setSteering(desiredSteering);

    if (mServoController) {
        // map from [-1.0:1.0] to actual servo range
        // TODO: all of this should happen in ServoController!
        if (mServoController->getInvertOutput())
            desiredSteering *= -1.0;
        desiredSteering = desiredSteering * (mServoController->getServoRange() / 2.0) + mServoController->getServoCenter();
        mServoController->requestSteering(desiredSteering);
    }

}

void CarMovementController::setDesiredSpeed(double desiredSpeed)
{
    MovementController::setDesiredSpeed(desiredSpeed);
    if (mMotorController)
        mMotorController->requestRPM(desiredSpeed*getSpeedToRPMFactor());
    else {
        static bool warnedOnce = false;
        if (!warnedOnce) {
            qDebug() << "WARNING: CarMovementController has no MotorController connection. Simulating movement."; // TODO: create explicitly simulated controller
            warnedOnce = true;
        }
        xyz_t currentVelocity = mCarState->getVelocity();
        currentVelocity.x = desiredSpeed;
        mCarState->setVelocity(currentVelocity);
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
