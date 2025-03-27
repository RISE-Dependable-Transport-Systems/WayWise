/*
 *     Copyright 2021 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Implements communication with a moving u-blox-based GNSS receiver
 */

#ifndef UBLOXROVER_H
#define UBLOXROVER_H

#include <QObject>
#include <QSharedPointer>
#include "ublox.h"
#include "gnssreceiver.h"
#include <QEventLoop>
#include <QTimer>

class UbloxRover : public GNSSReceiver
{
    Q_OBJECT
public:
    UbloxRover(QSharedPointer<VehicleState> vehicleState);
    bool connectSerial(const QSerialPortInfo &serialPortInfo);
    bool isSerialConnected();
    void writeRtcmToUblox(QByteArray data);
    void writeOdomToUblox(ubx_esf_datatype_enum dataType, uint32_t dataField, uint32_t timeTag = 0);
    void saveOnShutdown();
    void setUbloxVariant(UBLOX_VARIANT ubloxVariant) { mUbloxVariant = ubloxVariant; }
    void setDynamicModel(DynamicModel dynamicModel) { mDynamicModel = dynamicModel; }
    void setForceRecalibrateSensors(bool forceRecalibrateSensors) { mForceRecalibrateSensors = forceRecalibrateSensors; }
    void setGNSSMeasurementRate(int rate) { mGNSSMeasurementRate = rate; }
    void setNavPrioMessageRate(int rate) { mNavPrioMessageRate = rate; }
    void setSpeedDataInputRate(int rate) { mSpeedDataInputRate = rate; }
    void setPrintVerbose(bool printVerbose) { mPrintVerbose = printVerbose; }
    void setESFAlgAutoMntAlgOn(bool esfAlgAutoMntAlgOn) { mESFAlgAutoMntAlgOn = esfAlgAutoMntAlgOn; }

signals:
    void updatedGNSSPositionAndYaw(QSharedPointer<VehicleState> vehicleState, double distanceMoved, bool fused);
    void txNavPvt(const ubx_nav_pvt &pvt);
    void gotNmeaGga(const QByteArray& nmeaGgaStr);

private:
    bool configureUblox();
    void updateGNSSPositionAndYaw(const ubx_nav_pvt &pvt);
    void restoreBackedupConfiguration(int pollIntervalms = 1000, int maxPolls = 10);
    void createConfigurationBackup(int pollIntervalms = 1000, int maxPolls = 10);

    const int ms_per_day = 24 * 60 * 60 * 1000;
    Ublox mUblox;
    UBLOX_VARIANT mUbloxVariant = UBLOX_VARIANT::ZED_F9R;
    UBLOX_STATE mUbloxState = UBLOX_STATE::UNKNOWN;
    GNSS_FIX_TYPE mFixType = GNSS_FIX_TYPE::NO_FIX;

    DynamicModel mDynamicModel = DynamicModel::AUTOMOT;
    int mGNSSMeasurementRate = 5; // Hz
    int mNavPrioMessageRate = 10; // Hz
    int mSpeedDataInputRate = 10; // Hz
    bool mForceRecalibrateSensors = false;
    bool mESFAlgAutoMntAlgOn = false;
    bool mPrintVerbose = false;
};

#endif // UBLOXROVER_H
