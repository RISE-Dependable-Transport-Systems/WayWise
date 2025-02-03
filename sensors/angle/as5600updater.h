/*
 *     Copyright 2024 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Implementation of angle sensor specific for AS5600
 */

#ifndef AS5600UPDATER_H
#define AS5600UPDATER_H

#include "sensors/angle/anglesensorupdater.h"
#include "vehicles/truckstate.h"

extern "C" {
#include "external/pi-as5600/driver_as5600_basic.h"
}
#include <QTimer>

// connects to AS5600 via i2c bus and polls orientation data (angle) periodically
class AS5600Updater : public AngleSensorUpdater
{
public:
    AS5600Updater(QSharedPointer<VehicleState> vehicleState, double angleOffset=0);
    void printSensorInfo();
    virtual bool setUpdateIntervall(int pollIntervall_ms) override;
    virtual bool isConnected() override;

private:
    int mPollIntervall_ms = 50; // interval (ms) to read from AS5600,
    QTimer mPollTimer;
    bool mIsConnected = false;
    double angleOffset; // offset if the start angle is not zero

};

#endif // AS5600UPDATER_H
