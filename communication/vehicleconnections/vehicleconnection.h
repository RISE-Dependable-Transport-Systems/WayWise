/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Abstract class to communicate with a vehicle using a remote VehicleConnection
 */

#ifndef VEHICLECONNECTION_H
#define VEHICLECONNECTION_H

#include <QObject>
#include <QDebug>
#include "core/coordinatetransforms.h"
#include "vehicles/vehiclestate.h"
#include "sensors/camera/gimbal.h"
#include "autopilot/waypointfollower.h"

class VehicleConnection : public QObject
{
    Q_OBJECT
public:
    virtual void requestGotoENU(const xyz_t &xyz, bool changeAutopilotMode = false) = 0;
    virtual void requestVelocityAndYaw(const xyz_t &velocityENU, const double &yawDeg) = 0;
    virtual void setActuatorOutput(int index, float value) = 0;
    virtual void requestArm() = 0;
    virtual void requestDisarm() = 0;
    virtual void requestTakeoff() = 0;
    virtual void requestLanding() = 0;
    virtual void requestPrecisionLanding() = 0;
    virtual void requestReturnToHome() = 0;


    void setWaypointFollowerConnectionLocal(const QSharedPointer<WaypointFollower> &waypointFollower);
    bool hasWaypointFollowerConnectionLocal();
    bool isAutopilotActive();
    void restartAutopilot();
    void startAutopilot();
    void pauseAutopilot();
    void stopAutopilot();
    void clearRoute(int id = 0);
    void appendToRoute(const QList<PosPoint> &route, int id = 0);
    void setRoute(const QList<PosPoint> &route, int id = 0);

    QSharedPointer<VehicleState> getVehicleState() const;
    QSharedPointer<Gimbal> getGimbal() const;
    bool hasGimbal() const;

signals:
    void detectedGimbal(QSharedPointer<Gimbal> gimbal);
    void updatedBatteryState(float voltage, float percentRemaining);

protected:
    virtual bool isAutopilotActiveOnVehicle() {throw  std::logic_error("Function not implemented");};
    virtual void restartAutopilotOnVehicle() {throw  std::logic_error("Function not implemented");};
    virtual void startAutopilotOnVehicle() {throw  std::logic_error("Function not implemented");};
    virtual void pauseAutopilotOnVehicle() {throw  std::logic_error("Function not implemented");};
    virtual void stopAutopilotOnVehicle() {throw  std::logic_error("Function not implemented");};
    virtual void clearRouteOnVehicle(int id = 0) {Q_UNUSED(id) throw  std::logic_error("Function not implemented");};
    virtual void appendToRouteOnVehicle(const QList<PosPoint> &route, int id = 0) {Q_UNUSED(route )Q_UNUSED(id) throw  std::logic_error("Function not implemented");};

    QSharedPointer<VehicleState> mVehicleState;
    QSharedPointer<Gimbal> mGimbal;
    QSharedPointer<WaypointFollower> mWaypointFollower;


};

#endif // VEHICLECONNECTION_H
