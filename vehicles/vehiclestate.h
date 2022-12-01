/*
 *     Copyright 2012 Benjamin Vedder   benjamin@vedder.se
 *               2020 Marvin Damschen   marvin.damschen@ri.se
 *               2021 Lukas Wikander    lukas.wikander@astazero.com
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 * 
 * Specialisation of ObjectState for vehicles, storing all (dynamic and static) state
 */

#ifndef VEHICLESTATE_H
#define VEHICLESTATE_H

#include <QObject>
#include <QVector>
#include <QString>
#ifdef QT_GUI_LIB
#include <QPainter>
#endif

#include "core/pospoint.h"
#include "vehicles/objectstate.h"
#include <math.h>

class VehicleState : public ObjectState
{
    Q_OBJECT
public:
    // like MAVSDK FlightMode
    enum class FlightMode {
        Unknown,
        Ready,
        Takeoff,
        Hold,
        Mission,
        ReturnToLaunch,
        Land,
        Offboard,
        FollowMe,
        Manual,
        Altctl,
        Posctl,
        Acro,
        Stabilized,
        Rattitude
    };

    VehicleState(ObjectID_t id = 0, Qt::GlobalColor color = Qt::red);

    // Static state
    double getLength() const { return mLength; }
    void setLength(double length) { mLength = length; }
    double getWidth() const { return mWidth; }
    void setWidth(double width) { mWidth = width; }
    double getMinAcceleration() const { return mMinAcceleration; }
    void setMinAcceleration(double minAcceleration) { mMinAcceleration = minAcceleration; }
    double getMaxAcceleration() const { return mMaxAcceleration; }
    void setMaxAcceleration(double maxAcceleration) { mMaxAcceleration = maxAcceleration; }

    // Dynamic state
    virtual PosPoint getPosition(PosType type) const;
    virtual PosPoint getPosition() const override { return getPosition(PosType::simulated); }
    virtual void setPosition(PosPoint &point) override;
    virtual QTime getTime() const override { return mTime; }
    virtual void setTime(const QTime &time) override { mTime = time; }
    FlightMode getFlightMode() const;
    void setFlightMode(const FlightMode &flightMode);
    double getSteering() const;
    virtual void setSteering(double steering);
    PosPoint getHomePosition() const;
    void setHomePosition(const PosPoint &homePosition);
    bool getIsArmed() const;
    void setIsArmed(bool isArmed);
    void setAdaptivePurePursuitRadius(double speed);
    double getAdaptivePurePursuitRadius();

    void simulationStep(double dt_ms, PosType usePosType = PosType::simulated); // Take current state and simulate step forward for dt_ms milliseconds, update state accordingly
    virtual void updateOdomPositionAndYaw(double drivenDistance, PosType usePosType = PosType::odom) = 0;
    virtual double steeringCurvatureToSteering(double steeringCurvature) = 0;

    // For debugging and logging
    std::array<float, 3> getGyroscopeXYZ() const;
    void setGyroscopeXYZ(const std::array<float, 3> &gyroscopeXYZ);
    std::array<float, 3> getAccelerometerXYZ() const;
    void setAccelerometerXYZ(const std::array<float, 3> &accelerometerXYZ);


private:
    // Static state
    double mLength; // [m]
    double mWidth; // [m]
    // TODO: reasonable default values? set here or move?
    double mMinAcceleration = -5.0; // [m/s²]
    double mMaxAcceleration = 3.0; // [m/s²]

    // Dynamic state
    double mSteering = 0.0; // [-1.0:1.0]
    PosPoint mPositionBySource[(int)PosType::_LAST_];
    PosPoint mApGoal;
    QTime mTime;
    PosPoint mHomePosition;
    bool mIsArmed = false;
    FlightMode mFlightMode = FlightMode::Unknown;
    double mAdaptivePurePursuitRadius = 0;

    std::array<float,3> mGyroscopeXYZ = std::array<float,3>({0.0, 0.0, 0.0}); // [deg/s]
    std::array<float,3> mAccelerometerXYZ = std::array<float,3>({0.0, 0.0, 0.0}); // [g]
};

#endif // VEHICLESTATE_H
