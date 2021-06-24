#ifndef CARMOVEMENTCONTROLLER_H
#define CARMOVEMENTCONTROLLER_H

#include "movementcontroller.h"
#include "motorcontroller.h"
#include "servocontroller.h"
#include "carstate.h"
#include <QObject>
#include <QSharedPointer>

class CarMovementController : public MovementController
{
    Q_OBJECT
public:
    CarMovementController(QSharedPointer<CarState> vehicleState);
    // MovementController interface
    virtual void setDesiredSteering(double desiredSteering) override;
    virtual void setDesiredSpeed(double desiredSpeed) override;
    virtual void setDesiredSteeringCurvature(double desiredSteeringAngle) override;

    void setMotorController(const QSharedPointer<MotorController> motorController);
    void setServoController(const QSharedPointer<ServoController> servoController);

    double getSpeedToRPMFactor() const;
    void setSpeedToRPMFactor(double getSpeedToRPMFactor);


private:
    void updateVehicleState(double rpm, int tachometer, int tachometer_abs, double voltageInput, double temperature, int errorID);

    QSharedPointer<CarState> mCarState;
    QSharedPointer<MotorController> mMotorController;
    QSharedPointer<ServoController> mServoController;
    double mSpeedToRPMFactor = 4123.3; // default for Traxxas Slash VXL

};

#endif // CARMOVEMENTCONTROLLER_H
