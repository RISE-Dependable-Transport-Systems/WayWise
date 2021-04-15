#ifndef UBLOXROVER_H
#define UBLOXROVER_H

#include <QObject>
#include <QSharedPointer>
#include "ublox.h"
// TODO: move out?
extern "C" {
#include "../ext/Fusion/FusionBias.h"
#include "../ext/Fusion/FusionAhrs.h"
#include "../ext/Fusion/FusionCalibration.h"
}
#include "../vehiclestate.h"
class UbloxRover : public QObject
{
    Q_OBJECT
public:
    UbloxRover(QSharedPointer<VehicleState> vehicleState);
    bool connectSerial(const QSerialPortInfo &serialPortInfo);

signals:

private:
    bool configureUblox();
    void updateAHRS(const ubx_esf_meas &meas);

    Ublox mUblox;
    QSharedPointer<VehicleState> mVehicleState;

    const float mIMUSamplePeriod = 0.2f;
    FusionBias mFusionBias;
    FusionAhrs mFusionAhrs;
    FusionVector3 gyroscopeSensitivity = {{0.1f, 0.1f, 0.1f},}; // TODO: replace these values with actual sensitivity in degrees per second per lsb as specified in gyroscope datasheet
    FusionVector3 accelerometerSensitivity = {{0.03f, 0.03f, 0.03f},}; // TODO: replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet
};

#endif // UBLOXROVER_H
