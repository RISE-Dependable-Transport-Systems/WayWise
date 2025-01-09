#include "iso22133vehicleserver.h"
#include <QDebug>
#define MONR_RATE_MS 10
#define METERS_TO_MILLIMETERS 1000

iso22133VehicleServer::iso22133VehicleServer(
    QSharedPointer<VehicleState> vehicleState, const std::string &ip)
    : VehicleServer(vehicleState), ISO22133::TestObject(ip) {

    // Safety heartbeat
    mHeartbeat = false;
    mHeartbeatTimer.setSingleShot(true);
    connect(&mHeartbeatTimer, &QTimer::timeout, this,
            &iso22133VehicleServer::heartbeatTimeout);
    connect(this, &iso22133VehicleServer::resetHeartbeat, this,
            &iso22133VehicleServer::heartbeatReset);

    // Initialize required fields in MONR
    CartesianPosition pos;
    SpeedType spd;
    pos.xCoord_m = 0;
    pos.yCoord_m = 0;
    pos.zCoord_m = 0;
    pos.heading_rad = 0;
    pos.isHeadingValid = true;
    pos.isPositionValid = true;
    pos.isXcoordValid = true;
    pos.isYcoordValid = true;
    pos.isZcoordValid = true;
    spd.lateral_m_s = 0;
    spd.longitudinal_m_s = 0;
    spd.isLateralValid = true;
    spd.isLongitudinalValid = true;

    this->setPosition(pos);
    this->setSpeed(spd);

    // Publish vehicleState's info to Monr
    connect(&mSetMonrTimer, &QTimer::timeout, [this](){
        CartesianPosition pos;
        SpeedType spd;
        AccelerationType acc;
        DriveDirectionType drd;

        pos.xCoord_m = mVehicleState->getPosition(PosType::fused).getX();
        pos.yCoord_m = mVehicleState->getPosition(PosType::fused).getY();
        pos.zCoord_m = mVehicleState->getPosition(PosType::fused).getHeight(); // TODO: Should be negative??
        pos.isXcoordValid = true;
        pos.isYcoordValid = true;
        pos.isPositionValid = true;
        auto heading = mVehicleState->getPosition(PosType::fused).getYaw();
        // Convert [-180,180} to [0,360} deg
        if (heading < 0) { heading += 360; }
        pos.heading_rad = heading * M_PI / 180;
        pos.isHeadingValid = true;

        // Convert speed to longitudinal and lateral speed
        spd.longitudinal_m_s = mVehicleState->getSpeed();
        spd.lateral_m_s = 0;
        spd.isLongitudinalValid = true;
        spd.isLateralValid = true;

        // Note! mVehicleState in not filled with acceleration values at the moment
        acc.longitudinal_m_s2 = 0;
        acc.lateral_m_s2 = 0;
        acc.isLongitudinalValid = true;
        acc.isLateralValid = true;

        if (spd.longitudinal_m_s >= 0) {
            drd = DriveDirectionType::OBJECT_DRIVE_DIRECTION_FORWARD;
        } else {
            drd = DriveDirectionType::OBJECT_DRIVE_DIRECTION_BACKWARD;
        }
        
        setMonr(pos, spd, acc, drd);
    });   

    // Start writing state to Monr
    mSetMonrTimer.start(MONR_RATE_MS);
}

void iso22133VehicleServer::setUbloxRover(
    QSharedPointer<UbloxRover> ubloxRover) {
    if (!mGNSSReceiver.isNull()) {
        QSharedPointer<UbloxRover> mUbloxRover = qSharedPointerDynamicCast<UbloxRover>(mGNSSReceiver);
        if (mUbloxRover) {
            QObject::disconnect(
                mUbloxRover.get(), &UbloxRover::txNavPvt, this,
                &iso22133VehicleServer::updateRawGpsAndGpsInfoFromUbx);
        }
    }
    VehicleServer::setGNSSReceiver(ubloxRover);
    QObject::connect(ubloxRover.get(), &UbloxRover::txNavPvt, this,
                     &iso22133VehicleServer::updateRawGpsAndGpsInfoFromUbx);
}

void iso22133VehicleServer::setWaypointFollower(
    QSharedPointer<WaypointFollower> waypointFollower) {
    mWaypointFollower = waypointFollower;
    connect(this, &iso22133VehicleServer::startWaypointFollower,
            mWaypointFollower.get(), &WaypointFollower::startFollowingRoute);
    connect(this, &iso22133VehicleServer::pauseWaypointFollower,
            mWaypointFollower.get(), &WaypointFollower::stop);
    connect(this, &iso22133VehicleServer::resetWaypointFollower,
            mWaypointFollower.get(), &WaypointFollower::resetState);
    connect(this, &iso22133VehicleServer::clearRouteOnWaypointFollower,
            mWaypointFollower.get(), &WaypointFollower::clearRoute);
}

void iso22133VehicleServer::setMovementController(
    QSharedPointer<MovementController> movementController) {
    mMovementController = movementController;
}

void iso22133VehicleServer::setMonr(CartesianPosition pos, SpeedType spd, AccelerationType acc, DriveDirectionType drd) {
    this->setPosition(pos);
    this->setSpeed(spd);
    this->setAcceleration(acc);
    this->setDriveDirection(drd);
}

void iso22133VehicleServer::setManualControlMaxSpeed(
    double manualControlMaxSpeed_ms) {
    mManualControlMaxSpeed = manualControlMaxSpeed_ms;
} 

double iso22133VehicleServer::getManualControlMaxSpeed() const {
    return mManualControlMaxSpeed;
}

void iso22133VehicleServer::setStopCommands() {
    if (mWaypointFollower) {
        mWaypointFollower->stop();
    }

    if (mMovementController) {
        mMovementController->setDesiredSteering(0.0);
        mMovementController->setDesiredSpeed(0.0);
    }
}

void iso22133VehicleServer::heartbeatTimeout() {
    mHeartbeat = false;
    qDebug() << "iso22133VehicleServer: heartbeat timed out";
    setStopCommands();
}

void iso22133VehicleServer::heartbeatReset() {
    mHeartbeatTimer.start(mCountdown_ms);
    mHeartbeat = true;
}

void iso22133VehicleServer::handleAbort() {
    qDebug() << "iso22133VehicleServer: Abort recived!";
    setStopCommands();
    mWaypointFollower->resetState();
}

void iso22133VehicleServer::onOSEM(ObjectSettingsType &osem) {
    qDebug() << "Object Settings Received";
    setObjectSettings(osem);
    const llh_t llh = {osem.coordinateSystemOrigin.latitude_deg, osem.coordinateSystemOrigin.longitude_deg, osem.coordinateSystemOrigin.altitude_m};
    if (!mGNSSReceiver.isNull())
    {
        mGNSSReceiver->setEnuRef(llh);
    } else {
        qWarning() << "No UbloxRover set to receive llh!!";
    }
}

void iso22133VehicleServer::onTRAJ() {
    qDebug() << "Got onTRAJ signal, fetching new traj segments";

    std::vector<TrajectoryWaypointType> newTraj;
    newTraj = this->getTrajectory();
    if (this->getObjectSettings().testMode == TEST_MODE_ONLINE) {
        qDebug() << "Test mode is online planned, the mode is currently not "
                    "supported.";
    } else {
        qDebug() << "Test mode is preplanned, replacing existing trajectory";
        if (!mWaypointFollower.isNull()) {
            mWaypointFollower->clearRoute();
            QList<PosPoint> route;
            for (const auto &item : newTraj) {
                route.append(convertTrajPointToPosPoint(item));
            }
            mWaypointFollower->addRoute(route);
        } else {
            qDebug() << "iso22133VehicleServer: got new mission but no "
                        "WaypointFollower is set to receive it.";
        }
    }
}

PosPoint iso22133VehicleServer::convertTrajPointToPosPoint(
    const TrajectoryWaypointType &trajPoint) {
    PosPoint posPoint;
    posPoint.setX(trajPoint.pos.xCoord_m);
    posPoint.setY(trajPoint.pos.yCoord_m);
    posPoint.setHeight(trajPoint.pos.zCoord_m);
    // Set speed from long/lat speed components
    posPoint.setSpeed(sqrt(pow(trajPoint.spd.longitudinal_m_s, 2) +
                           pow(trajPoint.spd.lateral_m_s, 2)));

    return posPoint;
}

void iso22133VehicleServer::onSTRT(StartMessageType &) {
    qDebug() << "Object Starting";
    mVehicleState->setFlightMode(VehicleState::FlightMode::Mission);
    emit startWaypointFollower(false);
}

void iso22133VehicleServer::sendGpsOriginLlh(const llh_t &gpsOriginLlh) {
    // Not implemented
}

void iso22133VehicleServer::updateRawGpsAndGpsInfoFromUbx(
    const ubx_nav_pvt &pvt) {
    // Not implemented
}

void iso22133VehicleServer::setFollowPoint(
    QSharedPointer<FollowPoint> followPoint) {
    throw std::logic_error("iso22133VehicleServer::setFollowPoint(..) not implemented.");
}
