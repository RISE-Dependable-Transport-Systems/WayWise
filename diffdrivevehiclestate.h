#ifndef DIFFDRIVEVEHICLESTATE_H
#define DIFFDRIVEVEHICLESTATE_H

#include "vehiclestate.h"

class DiffDriveVehicleState : public VehicleState
{
public:
    DiffDriveVehicleState();

    virtual void simulationStep(double dt_ms) override;

    double getSpeedLeft() const;
    void setSpeedLeft(double getSpeedLeft);

    double getSpeedRight() const;
    void setSpeedRight(double getSpeedRight);

    double getSpeed() const override;

private:
    double mSpeedLeft;
    double mSpeedRight;
};

#endif // DIFFDRIVEVEHICLESTATE_H
