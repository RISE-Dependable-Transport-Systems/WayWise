#ifndef UBLOXROVER_H
#define UBLOXROVER_H

#include <QObject>
#include <QSharedPointer>
#include "ublox.h"
#include "sdvp_qtcommon/coordinatetransforms.h"
#include "../vehiclestate.h"

class UbloxRover : public QObject
{
    Q_OBJECT
public:
    UbloxRover(QSharedPointer<VehicleState> vehicleState);
    bool connectSerial(const QSerialPortInfo &serialPortInfo);
    bool isSerialConnected();
    llh_t getEnuRef() const;
    void setEnuRef(llh_t enuRef);
    void writeRtcmToUblox(QByteArray data);
    void writeOdoToUblox(ubx_esf_datatype_enum dataType, uint32_t dataField);
    void saveOnShutdown();
    void setIMUOrientationOffset(double roll_deg, double pitch_deg, double yaw_deg);

signals:
    void updatedGNSSPositionAndYaw(QSharedPointer<VehicleState> vehicleState, double distanceMoved, bool fused);

private:
    bool configureUblox();
    void updSosResponse(const ubx_upd_sos &sos);
    void updateGNSSPositionAndYaw(const ubx_nav_pvt &pvt);

    const int ms_per_day = 24 * 60 * 60 * 1000;

    llh_t mEnuReference;
    bool mEnuReferenceSet = false;

    Ublox mUblox;
    QSharedPointer<VehicleState> mVehicleState;
    struct {double rollOffset_deg, pitchOffset_deg, yawOffset_deg;} mIMUOrientationOffset;
};

#endif // UBLOXROVER_H
