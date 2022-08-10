#include "mavsdkvehicleserver.h"
#include <QDebug>
#include <future>

MavsdkVehicleServer::MavsdkVehicleServer(QSharedPointer<VehicleState> vehicleState)
{
    mVehicleState = vehicleState;

    mavsdk::Mavsdk::Configuration configuration(mavsdk::Mavsdk::Configuration::UsageType::Autopilot);
    mMavsdk.set_configuration(configuration);

    mavsdk::ConnectionResult result = mMavsdk.add_any_connection("udp://127.0.0.1:14540");
    if (result == mavsdk::ConnectionResult::Success)
        qDebug() << "MavsdkVehicleServer is listening...";

    std::shared_ptr<mavsdk::ServerComponent> serverComponent = mMavsdk.server_component_by_type(mavsdk::Mavsdk::ServerComponentType::Autopilot);

    // Create server plugins
    mTelemetryServer.reset(new mavsdk::TelemetryServer(serverComponent));
    mActionServer.reset(new mavsdk::ActionServer(serverComponent));
    mParamServer.reset(new mavsdk::ParamServer(serverComponent));
    mMissionRawServer.reset(new mavsdk::MissionRawServer(serverComponent));

    // These are needed for MAVSDK at the moment
    mParamServer->provide_param_int("CAL_ACC0_ID", 1);
    mParamServer->provide_param_int("CAL_GYRO0_ID", 1);
    mParamServer->provide_param_int("CAL_MAG0_ID", 1);
    mParamServer->provide_param_int("SYS_HITL", 0);
    mParamServer->provide_param_int("MIS_TAKEOFF_ALT", 0);

    // Allow the vehicle to change to auto mode (manual is always allowed) and arm, disable takeoff (only rover support for now)
    mActionServer->set_allowable_flight_modes({true, false, false});
    mActionServer->set_armable(true, true);
    mActionServer->set_allow_takeoff(false);

    // Publish vehicleState's telemetry info
    connect(&mPublishMavlinkTimer, &QTimer::timeout, [this](){
        mavsdk::TelemetryServer::PositionVelocityNed positionVelocityNed{{static_cast<float>(mVehicleState->getPosition(PosType::fused).getY()),
                                                                          static_cast<float>(mVehicleState->getPosition(PosType::fused).getX()),
                                                                          static_cast<float>(-mVehicleState->getPosition(PosType::fused).getHeight())},
                                                                         {static_cast<float>(mVehicleState->getVelocity().y),
                                                                          static_cast<float>(mVehicleState->getVelocity().x),
                                                                          static_cast<float>(-mVehicleState->getVelocity().z)}};

        mavsdk::TelemetryServer::VelocityNed velocity{static_cast<float>(mVehicleState->getVelocity().y),
                                                      static_cast<float>(mVehicleState->getVelocity().x),
                                                      static_cast<float>(-mVehicleState->getVelocity().z)};

        mavsdk::TelemetryServer::Heading heading{coordinateTransforms::yawENUtoNED(mVehicleState->getPosition(PosType::fused).getYaw())};

        mavsdk::TelemetryServer::Position homePositionLlh{};
        mavsdk::TelemetryServer::Position positionLlh{};
        // TODO: publish gpsOrigin and the following
//        mavsdk::TelemetryServer::RawGps rawGps{
//            0, 55.953251, -3.188267, 0, NAN, NAN, 0, NAN, 0, 0, 0, 0, 0, 0};
//        mavsdk::TelemetryServer::GpsInfo gpsInfo{11, mavsdk::TelemetryServer::FixType::Fix3D};

        if (!mUbloxRover.isNull()) {
            homePositionLlh = {mUbloxRover->getEnuRef().latitude, mUbloxRover->getEnuRef().longitude, static_cast<float>(mUbloxRover->getEnuRef().height), 0};

            llh_t fusedPosGlobal = coordinateTransforms::enuToLlh(mUbloxRover->getEnuRef(), {mVehicleState->getPosition(PosType::fused).getXYZ()});
            positionLlh = {fusedPosGlobal.latitude, fusedPosGlobal.longitude, static_cast<float>(fusedPosGlobal.height), 0};
        }

        mTelemetryServer->publish_position(positionLlh, velocity, heading);
        mTelemetryServer->publish_home(homePositionLlh);
        mTelemetryServer->publish_position_velocity_ned(positionVelocityNed);
        //mTelemetryServer->publish_raw_gps(rawGps, gpsInfo);
    });
    // TODO: publish rates
    mPublishMavlinkTimer.start(100);

    mActionServer->subscribe_flight_mode_change([this](mavsdk::ActionServer::Result res, mavsdk::ActionServer::FlightMode mode) {
       if  (res == mavsdk::ActionServer::Result::Success)
           mVehicleState->setFlightMode((VehicleState::FlightMode) mode);

           switch (mode) {
           case mavsdk::ActionServer::FlightMode::Mission:
               emit startWaypointFollower(false);
               break;
           case mavsdk::ActionServer::FlightMode::FollowMe:
               emit startFollowPoint();
               break;
           default:
               ;
           }

           if (mode != mavsdk::ActionServer::FlightMode::Mission &&
                   mode != mavsdk::ActionServer::FlightMode::FollowMe)
               if (mWaypointFollower->isActive())
                   emit pauseWaypointFollower();
    });

    mMissionRawServer->subscribe_incoming_mission([this](mavsdk::MissionRawServer::Result res, mavsdk::MissionRawServer::MissionPlan plan) {
        if (res != mavsdk::MissionRawServer::Result::Success)
            qDebug() << "MavsdkVehicleServer: failed to receive mission," << (int)res;
        else {
            qDebug() << "MavsdkVehicleServer: got new mission with" << plan.mission_items.size() << "items.";

            if (!mWaypointFollower.isNull()) {
                QList<PosPoint> route;
                for (const auto &item : plan.mission_items)
                    route.append(convertMissionItemToPosPoint(item));

                mWaypointFollower->addRoute(route);
            } else
                qDebug() << "MavsdkVehicleServer: got new mission but no WaypointFollower is set to receive it.";

        }
    });

    mMissionRawServer->subscribe_clear_all([this](uint32_t val) {
        Q_UNUSED(val)
        if (!mWaypointFollower.isNull())
            emit clearRouteOnWaypointFollower();
        else
            qDebug() << "MavsdkVehicleServer: got clear mission request but no WaypointFollower is set to receive it.";
    });

    mMissionRawServer->subscribe_current_item_changed([this](mavsdk::MissionRawServer::MissionItem item) {
        if (!mWaypointFollower.isNull() && item.seq == 0)
            emit resetWaypointFollower();
        else if (item.seq != 0)
            qDebug() << "Warning: jumping to seq ID in mission not implemented in MavsdkVehicleServer / WaypointFollower.";
    });

    // Safety heartbeat
    mHeartbeat = false;
    mHeartbeatTimer.setSingleShot(true);
    connect(&mHeartbeatTimer, &QTimer::timeout, this, &MavsdkVehicleServer::heartbeatTimeout);
    connect(this, &MavsdkVehicleServer::resetHeartbeat, this, &MavsdkVehicleServer::heartbeatReset);

    mMavsdk.intercept_incoming_messages_async([this](mavlink_message_t &message){
        if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) { // TODO: make sure this is actually for us
            if (!mHeartbeat) {
                qDebug() << "MavsdkVehicleServer: got heartbeat, timeout was reset.";
            }
            emit resetHeartbeat();
        } else if (!mHeartbeat)
            return false; // Drop incoming messages until heartbeat restored

//        qDebug() << "in:" << message.msgid << message.sysid << message.compid;

        switch (message.msgid) {
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
            mavlink_manual_control_t manual_control;
            mavlink_msg_manual_control_decode(&message, &manual_control);

            if (mVehicleState->getFlightMode() == VehicleState::FlightMode::Manual)
                handleManualControlMessage(manual_control);
            break;
        default:
            ;
        }

        return true;
    });

    mMavsdk.intercept_outgoing_messages_async([](mavlink_message_t &message){
        switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: // Fix some info in heartbeat s.th. MAVSDK / ControlTower detects vehicle correctly
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&message, &heartbeat);
            heartbeat.type = MAV_TYPE_GROUND_ROVER;
            heartbeat.autopilot = WAYWISE_MAVLINK_AUTOPILOT_ID;
            heartbeat.base_mode |= MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; // Note: behave like PX4...
            mavlink_msg_heartbeat_encode(message.sysid, message.compid, &message, &heartbeat);
//            qDebug() << "MAVLINK_MSG_ID_HEARTBEAT:" << heartbeat.type << heartbeat.autopilot << heartbeat.base_mode << heartbeat.custom_mode << heartbeat.system_status << heartbeat.mavlink_version;
            break;
//        case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
//            mavlink_autopilot_version_t autopilot_version;
//            mavlink_msg_autopilot_version_decode(&message, &autopilot_version);
//            qDebug() << "MAVLINK_MSG_ID_AUTOPILOT_VERSION:" << autopilot_version.capabilities << autopilot_version.flight_sw_version << autopilot_version.middleware_sw_version <<
//                        autopilot_version.os_sw_version << autopilot_version.board_version << autopilot_version.flight_custom_version << autopilot_version.middleware_custom_version <<
//                        autopilot_version.os_custom_version << autopilot_version.vendor_id << autopilot_version.product_id << autopilot_version.uid << autopilot_version.uid2;
//            break;
        default: ;
//            qDebug() << "out:" << message.msgid;
        }
        return true;

    });
}

void MavsdkVehicleServer::heartbeatTimeout() {
    mHeartbeat = false;
    qDebug() << "MavsdkVehicleServer: heartbeat timed out";

    if (mWaypointFollower) {
        mWaypointFollower->stop();
     }

     if (mMovementController) {
              mMovementController->setDesiredSteering(0.0);
              mMovementController->setDesiredSpeed(0.0);
     }
}

void MavsdkVehicleServer::heartbeatReset()
{
    mHeartbeatTimer.start(mCountdown_ms);
    mHeartbeat = true;
}

void MavsdkVehicleServer::setUbloxRover(QSharedPointer<UbloxRover> ubloxRover)
{
    mUbloxRover = ubloxRover;
}

void MavsdkVehicleServer::setWaypointFollower(QSharedPointer<WaypointFollower> waypointFollower)
{
    mWaypointFollower = waypointFollower;
    connect(this, &MavsdkVehicleServer::startWaypointFollower, mWaypointFollower.get(), &WaypointFollower::startFollowingRoute);
    connect(this, &MavsdkVehicleServer::pauseWaypointFollower, mWaypointFollower.get(), &WaypointFollower::stop);
    connect(this, &MavsdkVehicleServer::resetWaypointFollower, mWaypointFollower.get(), &WaypointFollower::resetState);
    connect(this, &MavsdkVehicleServer::clearRouteOnWaypointFollower, mWaypointFollower.get(), &WaypointFollower::clearRoute);
    connect(this, &MavsdkVehicleServer::startFollowPoint, mWaypointFollower.get(), &WaypointFollower::startFollowPoint);
}

void MavsdkVehicleServer::setMovementController(QSharedPointer<MovementController> movementController)
{
    mMovementController = movementController;
}

PosPoint MavsdkVehicleServer::convertMissionItemToPosPoint(const mavsdk::MissionRawServer::MissionItem &item)
{
    PosPoint routePoint;

    routePoint.setX(item.x / 10e4);
    routePoint.setY(item.y / 10e4);
    routePoint.setHeight(item.z);
     // TODO: does not follow MAV_CMD_NAV_WAYPOINT definition
    routePoint.setSpeed(item.param1);
    routePoint.setAttributes(item.param2);

//    qDebug() << routePoint.getX() << routePoint.getY() << routePoint.getHeight();

    return routePoint;
}

void MavsdkVehicleServer::handleManualControlMessage(mavlink_manual_control_t manualControl)
{
    if (!mMovementController.isNull()) {
        mVehicleState->setFlightMode(VehicleState::FlightMode::Manual);

        if (!mWaypointFollower.isNull() && mWaypointFollower->isActive()) {
            qDebug() << "MavsdkVehicleServer: WaypointFollower stopped by manual control input.";
            mWaypointFollower->stop();
        }

        mMovementController->setDesiredSpeed((manualControl.x / 1000.0) * mManualControlMaxSpeed);
        mMovementController->setDesiredSteering(manualControl.r / 1000.0);
    } else
        qDebug() << "Warning: MavsdkVehicleServer got manual control message, but has no MovementController to talk to.";
}

void MavsdkVehicleServer::setManualControlMaxSpeed(double manualControlMaxSpeed_ms)
{
    mManualControlMaxSpeed = manualControlMaxSpeed_ms;
}
