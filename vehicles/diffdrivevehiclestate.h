#ifndef DIFFDRIVEVEHICLESTATE_H
#define DIFFDRIVEVEHICLESTATE_H

#include "vehicles/vehiclestate.h"

class DiffDriveVehicleState : public VehicleState
{
public:
    DiffDriveVehicleState();

    virtual void setSteering(double steering) override;
    double getSpeed() const override;
    virtual void setSpeed(double speed) override;

    virtual void updateOdomPositionAndYaw(double drivenDistance, PosType usePosType = PosType::odom) override;
    virtual double steeringCurvatureToSteering(double steeringCurvature) override;

    double getSpeedLeft() const;
    void setSpeedLeft(double getSpeedLeft);

    double getSpeedRight() const;
    void setSpeedRight(double getSpeedRight);


private:
    double mSpeedLeft;
    double mSpeedRight;
};

#endif // DIFFDRIVEVEHICLESTATE_H
