/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Implementation of IMUOrientationUpdater for Bosch BNO055 IMUs
 */

#ifndef BNO055ORIENTATIONUPDATER_H
#define BNO055ORIENTATIONUPDATER_H

#include "sensors/imu/imuorientationupdater.h"
extern "C" {
#include "external/pi-bno055/getbno055.h"
}
#include <QTimer>

// connects to BNO055 via i2c bus and polls orientation data (euler angles) periodically
class BNO055OrientationUpdater : public IMUOrientationUpdater
{
public:
    BNO055OrientationUpdater(QSharedPointer<VehicleState> vehicleState, QString i2cBus = "/dev/i2c-1");
    void printBNO055Info();

    virtual bool setUpdateIntervall(int pollIntervall_ms) override;

private:
    const QString mBNO055I2CAddress = "0x28";
    int mPollIntervall_ms = 20;
    QTimer mPollTimer;
};

#endif // BNO055ORIENTATIONUPDATER_H
