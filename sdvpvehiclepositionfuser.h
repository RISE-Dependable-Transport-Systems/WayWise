#ifndef SDVPVEHICLEPOSITIONFUSER_H
#define SDVPVEHICLEPOSITIONFUSER_H

#include <QObject>
#include <QSharedPointer>
#include <sdvp_qtcommon/vehiclestate.h>

class SDVPVehiclePositionFuser : public QObject
{
    Q_OBJECT
public:
    explicit SDVPVehiclePositionFuser(QObject *parent = nullptr);
    void correctPositionAndYawGNSS(QSharedPointer<VehicleState> vehicleState, double distanceMoved, bool fused);
    void correctPositionAndYawOdom(QSharedPointer<VehicleState> vehicleState, double distanceDriven);
    void correctPositionAndYawIMU(QSharedPointer<VehicleState> vehicleState);

    void setPosGNSSxyGain(double posGNSSxyGain);
    void setPosGNSSyawGain(double posGNSSyawGain);

signals:

private:
    struct PosSample {
        QPointF posXY;
        double yaw;
      QTime timestamp;
    };

    PosSample getClosestPosFusedSampleInTime(int msecsSinceStartOfDay);

    double mPosIMUyawOffset = 0.0;
    bool mPosGNSSisFused = false; // use GNSS pos as "fused" pos when true, e.g., F9R
    double mPosGNSSxyGain = 0.05;
    double mPosGNSSyawGain = 1.0;
    double mPosOdomLastDistanceDriven = std::numeric_limits<double>::min();
    static constexpr double BIG_DISTANCE_ERROR_m = 50.0;

    static constexpr int POSFUSED_HISTORY_SIZE = 128;
    PosSample mPosFusedHistory[POSFUSED_HISTORY_SIZE];
    int mPosFusedHistoryCurrentIdx = 0;
};

#endif // SDVPVEHICLEPOSITIONFUSER_H
