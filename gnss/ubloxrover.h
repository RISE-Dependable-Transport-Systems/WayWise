#ifndef UBLOXROVER_H
#define UBLOXROVER_H

#include <QObject>
#include <QSharedPointer>
#include "ublox.h"
#include "sdvp_qtcommon/coordinatetransforms.h"
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
    void setEnuRef(llh_t enuRef);
    void writeRtcmToUblox(QByteArray data);
    void writeOdoToUblox(ubx_esf_datatype_enum dataType, uint32_t dataField);
    void saveOnShutdown();

signals:
    void updatedGNSSPositionAndYaw(QSharedPointer<VehicleState> vehicleState, bool fused);
    void updatedIMUOrientation(QSharedPointer<VehicleState> vehicleState);

private:
    bool configureUblox();
    void updateAHRS(const ubx_esf_meas &meas);
    void updSosResponse(const ubx_upd_sos &sos);
    void updateGNSSPositionAndYaw(const ubx_nav_pvt &pvt);

    llh_t mEnuReference;
    bool mEnuReferenceSet = false;

    Ublox mUblox;
    QSharedPointer<VehicleState> mVehicleState;

    const float mIMUSamplePeriod = 0.2f;
    FusionBias mFusionBias;
    FusionAhrs mFusionAhrs;
    FusionVector3 gyroscopeSensitivity = {{0.1f, 0.1f, 0.1f},}; // TODO: replace these values with actual sensitivity in degrees per second per lsb as specified in gyroscope datasheet
    FusionVector3 accelerometerSensitivity = {{0.03f, 0.03f, 0.03f},}; // TODO: replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet
};

#endif // UBLOXROVER_H
