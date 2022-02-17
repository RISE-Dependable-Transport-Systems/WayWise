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

    void setPosGNSSxyStaticGain(double posGNSSxyStaticGain);
    void setPosGNSSyawGain(double posGNSSyawGain);

    double getPosGNSSxyDynamicGain() const;
    void setPosGNSSxyDynamicGain(double posGNSSxyDynamicGain);

signals:

private:
    struct PosSample {
        QPointF posXY;
        double yaw;
        QTime timestamp;
    };

    double getMaxSignedStepFromValueTowardsGoal(double value, double goal, double maxStepSize);

    void samplePosFused(const PosPoint &posFused);
    PosSample getClosestPosFusedSampleInTime(QTime timeUTC);

    double mPosIMUyawOffset = 0.0;
    bool mPosGNSSisFused = false; // use GNSS pos as "fused" pos when true, e.g., F9R
    double mPosGNSSxyStaticGain = 0.05;
    double mPosGNSSxyDynamicGain = 0.1;
    double mPosGNSSyawGain = 1.0;
    double mPosOdomDistanceDrivenSinceGNSSupdate = std::numeric_limits<double>::min();
    static constexpr double BIG_DISTANCE_ERROR_m = 50.0;

    static constexpr int POSFUSED_HISTORY_SIZE = 128;
    PosSample mPosFusedHistory[POSFUSED_HISTORY_SIZE];
    int mPosFusedHistoryCurrentIdx = 0;
};

#endif // SDVPVEHICLEPOSITIONFUSER_H
