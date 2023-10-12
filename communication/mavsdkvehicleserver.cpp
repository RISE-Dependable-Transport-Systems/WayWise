/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *               2022 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#include "mavsdkvehicleserver.h"
#include <QDebug>
#include <future>
#include <QMetaMethod>
#include <algorithm>
#include <chrono>
#include <QMetaType>
#include "WayWise/logger/logger.h"

MavsdkVehicleServer::MavsdkVehicleServer(QSharedPointer<VehicleState> vehicleState)
{
    connect(&Logger::getInstance(), &Logger::logSent, this, &MavsdkVehicleServer::on_logSent);

    mVehicleState = vehicleState;

    mavsdk::Mavsdk::Configuration configuration(mavsdk::Mavsdk::Configuration::UsageType::Autopilot);
    mMavsdk.set_configuration(configuration);

    std::shared_ptr<mavsdk::ServerComponent> serverComponent = mMavsdk.server_component_by_type(mavsdk::Mavsdk::ServerComponentType::Autopilot);

    // Create server plugins
    MavlinkParameterServer::initialize(serverComponent);
    mParameterServer = MavlinkParameterServer::getInstance();
    mTelemetryServer.reset(new mavsdk::TelemetryServer(serverComponent));
    mActionServer.reset(new mavsdk::ActionServer(serverComponent));
    mMissionRawServer.reset(new mavsdk::MissionRawServer(serverComponent));

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

        if (!mUbloxRover.isNull()) {
            // publish gpsOrigin
            sendGpsOriginLlh(mUbloxRover->getEnuRef());

            //TODO: homePositionLlh should not be EnuRef
            homePositionLlh = {mUbloxRover->getEnuRef().latitude, mUbloxRover->getEnuRef().longitude, static_cast<float>(mUbloxRover->getEnuRef().height), 0};

            llh_t fusedPosGlobal = coordinateTransforms::enuToLlh(mUbloxRover->getEnuRef(), {mVehicleState->getPosition(PosType::fused).getXYZ()});
            positionLlh = {fusedPosGlobal.latitude, fusedPosGlobal.longitude, static_cast<float>(fusedPosGlobal.height), 0};
        }

        mTelemetryServer->publish_position(positionLlh, velocity, heading);
        mTelemetryServer->publish_home(homePositionLlh);
        mTelemetryServer->publish_position_velocity_ned(positionVelocityNed);
        mTelemetryServer->publish_raw_gps(mRawGps, mGpsInfo);
    });    

    mActionServer->subscribe_flight_mode_change([this](mavsdk::ActionServer::Result res, mavsdk::ActionServer::FlightMode mode) {
        if  (res == mavsdk::ActionServer::Result::Success) {
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

    // --- Things that we should implement/fix in MAVSDK follow from here :-)
    // Create mavlinkpassthrough plugin (TODO: should not be used on vehicle side...)
    mMavsdk.subscribe_on_new_system([this](){
        mMavlinkPassthrough.reset(new mavsdk::MavlinkPassthrough(mMavsdk.systems().at(0)));

        mMavlinkPassthrough->subscribe_message(MAVLINK_MSG_ID_COMMAND_LONG, [this](const mavlink_message_t &message) {
            switch (mavlink_msg_command_long_get_command(&message)) {
            case MAV_CMD_DO_SET_MISSION_CURRENT: // used to switch between multiple autopilots
                static const QMetaMethod switchAutopilotIDSignal = QMetaMethod::fromSignal(&MavsdkVehicleServer::switchAutopilotID);
                if (isSignalConnected(switchAutopilotIDSignal))
                    emit switchAutopilotID(mavlink_msg_command_long_get_param3(&message));
                else
                    qDebug() << "Warning: got request to change autopilot id, but switchAutopilotID(..) signal is not connected.";
                break;
            case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                auto param2Value = mavlink_msg_command_long_get_param2(&message);
                if (param2Value == 1) {
                    mavResult(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, MAV_RESULT_ACCEPTED);
                    emit shutdownOrRebootOnboardComputer(false);
                } else if (param2Value == 2) {
                    mavResult(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, MAV_RESULT_ACCEPTED);
                    emit shutdownOrRebootOnboardComputer(true);
                } else
                    mavResult(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, MAV_RESULT_DENIED);
                break;
            }
        });

        // Handle RTCM data
        mMavlinkPassthrough->subscribe_message(MAVLINK_MSG_ID_GPS_RTCM_DATA, [this](const mavlink_message_t &message) {
            mavlink_gps_rtcm_data_t mavRtcmData;
            mavlink_msg_gps_rtcm_data_decode(&message, &mavRtcmData);
            static const uint8_t maxMavlinkMessageLength = 180;
            static QByteArray fragments[4];
            static uint8_t lastFragment = 0;
            static uint8_t sequenceIdBuffer[4];

            QByteArray rtcmData;
            if ((mavRtcmData.flags & 0x1) != 0) {// LSB set indicates message is fragmented
                uint8_t sequenceId = ((mavRtcmData.flags >> 3) & 0x1F);  // 5 bits are sequence id
                uint8_t fragmentId = (mavRtcmData.flags >> 1) & 0x3;

                if (mavRtcmData.len < maxMavlinkMessageLength) // Last fragment
                    lastFragment = fragmentId;
                sequenceIdBuffer[fragmentId] = sequenceId;
                fragments[fragmentId].resize(std::min(mavRtcmData.len, maxMavlinkMessageLength));
                memcpy(fragments[fragmentId].data(), mavRtcmData.data, std::min(mavRtcmData.len, maxMavlinkMessageLength));

                if (lastFragment > 0 && std::all_of(sequenceIdBuffer, sequenceIdBuffer + lastFragment, [sequenceId](uint8_t x) { return x == sequenceId;})) { // Buffer is reconstructed with same sequence id
                    for (int i = 0; i <= lastFragment; i++)
                        rtcmData.append(fragments[i]);
                    emit rxRtcmData(rtcmData);
                    lastFragment = 0;
                }
            } else {
                rtcmData.resize(mavRtcmData.len);
                memcpy(rtcmData.data(), mavRtcmData.data, mavRtcmData.len);
                emit rxRtcmData(rtcmData);
            }
        });

        // Publish autopilot radius
        connect(&mPublishMavlinkTimer, &QTimer::timeout, [this](){
            mavlink_message_t mavAutopilotRadiusmMsg;
            mavlink_named_value_float_t autopilotRadius;
            memset(&autopilotRadius, 0, sizeof(mavlink_named_value_float_t));
            std::chrono::milliseconds counter;
            autopilotRadius.time_boot_ms = counter.count();
            autopilotRadius.value = mVehicleState->getAutopilotRadius();
            strcpy(autopilotRadius.name, "AR");
            mavlink_msg_named_value_float_encode(mMavlinkPassthrough->get_our_sysid(), mMavlinkPassthrough->get_our_compid(), &mavAutopilotRadiusmMsg, &autopilotRadius);
            if (mMavlinkPassthrough->send_message(mavAutopilotRadiusmMsg) != mavsdk::MavlinkPassthrough::Result::Success)
                qDebug() << "Warning: could not send autopilot radius via MAVLINK.";
        });
    });

    mMavsdk.intercept_outgoing_messages_async([this](mavlink_message_t &message){
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
        case MAVLINK_MSG_ID_PARAM_VALUE: // This message is sent when a parameter has been changed
            mavlink_param_value_t parameter;
            mavlink_msg_param_value_decode(&message, &parameter);
            if (parameter.param_type == 9) // Float
                mParameterServer->updateParameter(parameter.param_id, parameter.param_value);
        default: ;
//            qDebug() << "out:" << message.msgid;
        }
        return true;

    });

    // Start publishing status on MAVLink (TODO: rate?)
    mPublishMavlinkTimer.start(100);

    mavsdk::ConnectionResult result = mMavsdk.add_any_connection("udp://127.0.0.1:14540");
    if (result == mavsdk::ConnectionResult::Success)
        qDebug() << "MavsdkVehicleServer is listening...";
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
    if (!mUbloxRover.isNull())
        disconnect(mUbloxRover.get(), &UbloxRover::txNavPvt, this, &MavsdkVehicleServer::updateRawGpsAndGpsInfoFromUbx);

    mUbloxRover = ubloxRover;
    connect(mUbloxRover.get(), &UbloxRover::txNavPvt, this, &MavsdkVehicleServer::updateRawGpsAndGpsInfoFromUbx);
}

void MavsdkVehicleServer::updateRawGpsAndGpsInfoFromUbx(const ubx_nav_pvt &pvt) {
    mRawGps.timestamp_us = pvt.i_tow*1000; // TODO
    mRawGps.latitude_deg = pvt.lat;
    mRawGps.longitude_deg = pvt.lon;
    mRawGps.absolute_altitude_m = pvt.height;
    mRawGps.hdop = pvt.p_dop; // hdop <= pdop
    mRawGps.vdop = pvt.p_dop; // vdop <= pdop
    mRawGps.velocity_m_s = sqrt(pvt.vel_n*pvt.vel_n + pvt.vel_e*pvt.vel_e + pvt.vel_d*pvt.vel_d);
    mRawGps.cog_deg = pvt.head_mot;
    mRawGps.altitude_ellipsoid_m = pvt.height;
    mRawGps.horizontal_uncertainty_m = pvt.h_acc;
    mRawGps.vertical_uncertainty_m = pvt.v_acc;
    mRawGps.velocity_uncertainty_m_s = pvt.s_acc;
    mRawGps.heading_uncertainty_deg = pvt.head_acc;
    mRawGps.yaw_deg = pvt.head_veh;

    mGpsInfo.num_satellites = pvt.num_sv;
    mGpsInfo.fix_type = mavsdk::TelemetryServer::FixType::NoFix;
    if (pvt.fix_type == 2)
        mGpsInfo.fix_type = mavsdk::TelemetryServer::FixType::Fix2D;
    else if (pvt.fix_type == 3 || pvt.fix_type == 4) {
        mGpsInfo.fix_type= mavsdk::TelemetryServer::FixType::Fix3D;
        if (pvt.carr_soln == 1)
            mGpsInfo.fix_type= mavsdk::TelemetryServer::FixType::RtkFloat;
        else if (pvt.carr_soln == 2)
            mGpsInfo.fix_type= mavsdk::TelemetryServer::FixType::RtkFixed;
    }

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

void MavsdkVehicleServer::mavResult(const uint16_t command, MAV_RESULT result)
{
        mavlink_message_t ack;
        ack = mMavlinkPassthrough->make_command_ack_message(mMavlinkPassthrough->get_target_sysid(), mMavlinkPassthrough->get_target_compid(), command, result);
        mMavlinkPassthrough->send_message(ack);
};

void MavsdkVehicleServer::sendGpsOriginLlh(const llh_t &gpsOriginLlh)
{
    if (mMavlinkPassthrough == nullptr)
        return;

    mavlink_message_t mavGpsGlobalOriginMsg;
    mavlink_gps_global_origin_t mavGpsGlobalOrigin;
    memset(&mavGpsGlobalOrigin, 0, sizeof(mavlink_gps_global_origin_t));

    mavGpsGlobalOrigin.latitude = (int) (gpsOriginLlh.latitude * 1e7);
    mavGpsGlobalOrigin.longitude = (int) (gpsOriginLlh.longitude * 1e7);
    mavGpsGlobalOrigin.altitude = (int) (gpsOriginLlh.height * 1e3);

    mavlink_msg_gps_global_origin_encode(mMavlinkPassthrough->get_our_sysid(), mMavlinkPassthrough->get_our_compid(), &mavGpsGlobalOriginMsg, &mavGpsGlobalOrigin);
    if (mMavlinkPassthrough->send_message(mavGpsGlobalOriginMsg) != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: could not send GPS_GLOBAL_ORIGIN via MAVLINK.";
};

void MavsdkVehicleServer::on_logSent(const QString& message, const quint8& severity)
{
    struct logQueueItem {
        QString log;
        quint16 id;
        quint8 severity;
    };

    static QVector<logQueueItem> logQueue;

    logQueueItem item;

    item.log = message;
    item.severity = severity;

    logQueue.push_back(item);

    if(!mMavlinkPassthrough)
        return;

    for(const logQueueItem &item : logQueue) {

        static quint16 idCounter = 1;

        const int chunkSize = MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 8; // STATUSTEXT supports up to char[50], -8 adjustment to fit MAVLink header
        int numChunks = (item.log.length() + chunkSize - 1) / chunkSize;

        if(item.log.length() % 42 == 0)  // need extra message for null terminator, so to not overwrite any message character
            numChunks++;

        for(int chunkIndex = 0; chunkIndex < numChunks; chunkIndex++) {
            QString chunk = item.log.mid(chunkIndex * chunkSize, chunkSize);
            QByteArray chunkBytes = chunk.toUtf8();

            mavlink_statustext_t statusText;    // send text messages to ground-station or other MAVLink-enabled systems
            memset(&statusText, 1, sizeof(statusText));

            statusText.severity = item.severity;

            qstrcpy(statusText.text, chunkBytes.constData());

            if(chunkIndex == numChunks - 1)
                statusText.text[49] = '\0'; // ensure null terminated

            if(numChunks > 1)
                statusText.id = idCounter;
            else
                statusText.id = 0;  // message can be omitted directly

            statusText.chunk_seq = chunkIndex;

            mavlink_message_t mavLogMsg;
            mavlink_msg_statustext_encode(mMavlinkPassthrough->get_our_sysid(), mMavlinkPassthrough->get_our_compid(), &mavLogMsg, &statusText);

            if (mMavlinkPassthrough->send_message(mavLogMsg) != mavsdk::MavlinkPassthrough::Result::Success)
                qWarning() << "Could not send log output via MAVLINK.";
        }

        if(idCounter == 65535)  // overflow avoidance
            idCounter = 1;
        else
            idCounter++;
    }

    logQueue.clear();
}
