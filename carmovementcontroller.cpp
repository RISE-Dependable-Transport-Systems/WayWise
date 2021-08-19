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

    if (mServoController) {
        // map from [-1.0:1.0] to actual servo range
        // TODO: all of this should happen in ServoController!
        if (mServoController->getInvertOutput())
            desiredSteering *= -1.0;
        desiredSteering = desiredSteering * (mServoController->getServoRange() / 2.0) + mServoController->getServoCenter();
        mServoController->requestSteering(desiredSteering);
    }

    // update vehicleState in any case (we do not expect feedback from servo)
    mCarState->setSteering(desiredSteering);
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
        mCarState->setSpeed(desiredSpeed);
    }
}

void CarMovementController::setDesiredSteeringCurvature(double desiredSteeringCurvature)
{
    double steeringAngle_rad = atan(mCarState->getAxisDistance() * desiredSteeringCurvature);
    if (abs(steeringAngle_rad) > mCarState->getMaxSteeringAngle())
        steeringAngle_rad = mCarState->getMaxSteeringAngle() * ((steeringAngle_rad > 0) ? 1.0 : -1.0);

    setDesiredSteering(steeringAngle_rad / mCarState->getMaxSteeringAngle());
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
    Q_UNUSED(tachometer) // TODO: use for odometry
    Q_UNUSED(tachometer_abs) // TODO: use for odometry
    Q_UNUSED(voltageInput)
    Q_UNUSED(temperature)
    Q_UNUSED(errorID)
    getVehicleState().dynamicCast<CarState>()->setSpeed(rpm/getSpeedToRPMFactor());
}
