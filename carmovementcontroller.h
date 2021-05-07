#ifndef CARMOVEMENTCONTROLLER_H
#define CARMOVEMENTCONTROLLER_H

#include "movementcontroller.h"
#include "carstate.h"
#include <QObject>
#include <QSharedPointer>

class CarMovementController : public MovementController
{
    Q_OBJECT
public:
    CarMovementController(QSharedPointer<CarState> vehicleState);
private:

    // MovementController interface
public:
    virtual void setDesiredSteering(double desiredSteering) override;
    virtual void setDesiredSpeed(double desiredSpeed) override;
};

#endif // CARMOVEMENTCONTROLLER_H
