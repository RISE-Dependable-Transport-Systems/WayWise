#include "mavsdkvehicleconnection.h"
#include "vehicles/copterstate.h"
#include <QDebug>
#include <QDateTime>

MavsdkVehicleConnection::MavsdkVehicleConnection(std::shared_ptr<mavsdk::System> system)
{
    mSystem = system;

    // TODO: create respective class depending on MAV_TYPE (but not accessible in MAVSDK?!)
    mVehicleType = MAV_TYPE::MAV_TYPE_QUADROTOR;
    switch (mVehicleType) {
    case MAV_TYPE_QUADROTOR:
        mVehicleState = QSharedPointer<CopterState>::create(mSystem->get_system_id());
        mVehicleState->setName("Copter " + QString::number(mSystem->get_system_id()));
        break;
    case MAV_TYPE_GROUND_ROVER:
        // TODO: car or diffdrive?
        qDebug() << "MAV_TYPE_GROUND_ROVER not implemented.";
        break;
    default:
        break;
    }

    // Set up telemetry plugin
    mTelemetry.reset(new mavsdk::Telemetry(mSystem));

    mTelemetry->subscribe_armed([this](bool isArmed) {
       mVehicleState->setIsArmed(isArmed);
    });

    mTelemetry->subscribe_home([this](mavsdk::Telemetry::Position position) {
        llh_t llh = {position.latitude_deg, position.longitude_deg, position.absolute_altitude_m};
        xyz_t xyz = coordinateTransforms::llhToEnu(mEnuReference, llh);

        auto homePos = mVehicleState->getHomePosition();
        homePos.setX(xyz.x);
        homePos.setY(xyz.y);
        homePos.setHeight(xyz.z);
        mVehicleState->setHomePosition(homePos);

        emit gotVehicleHomeLlh({position.latitude_deg, position.longitude_deg, position.absolute_altitude_m});

        // hook onto this callback to poll gps origin
        mTelemetry->get_gps_global_origin_async([this](mavsdk::Telemetry::Result result, mavsdk::Telemetry::GpsGlobalOrigin gpsGlobalOrigin){
            if (result == mavsdk::Telemetry::Result::Success)
                mGpsGlobalOrigin = {gpsGlobalOrigin.latitude_deg, gpsGlobalOrigin.longitude_deg, gpsGlobalOrigin.altitude_m};
        });
    });

    mTelemetry->subscribe_position([this](mavsdk::Telemetry::Position position) {
        llh_t llh = {position.latitude_deg, position.longitude_deg, position.absolute_altitude_m};
        xyz_t xyz = coordinateTransforms::llhToEnu(mEnuReference, llh);

        auto pos = mVehicleState->getPosition();
        pos.setX(xyz.x);
        pos.setY(xyz.y);
        pos.setHeight(xyz.z);

        mVehicleState->setPosition(pos);
    });

    mTelemetry->subscribe_attitude_quaternion([this](mavsdk::Telemetry::Quaternion q) {
        auto pos = mVehicleState->getPosition();
        pos.setYaw(atan2f(q.w * q.z + q.x * q.y, 0.5 - (q.y * q.y + q.z * q.z))); // extract yaw from quaternion
        mVehicleState->setPosition(pos);
    });

    mTelemetry->subscribe_velocity_ned([this](mavsdk::Telemetry::VelocityNed velocity) {
        VehicleState::Velocity velocityCopter {velocity.east_m_s, velocity.north_m_s, -velocity.down_m_s};
        mVehicleState->setVelocity(velocityCopter);
    });

    if (mVehicleType == MAV_TYPE::MAV_TYPE_QUADROTOR)
        mTelemetry->subscribe_landed_state([this](mavsdk::Telemetry::LandedState landedState) {
           mVehicleState.dynamicCast<CopterState>()->setLandedState(static_cast<CopterState::LandedState>(landedState));
        });

    // Set up action plugin
    mAction.reset(new mavsdk::Action(mSystem));

    // Set up praram plugin
    mParam.reset(new mavsdk::Param(mSystem));
    // Precision Landing: set required target tracking accuracy for starting approach
    if (mParam->set_param_float("PLD_HACC_RAD", 5.0) != mavsdk::Param::Result::Success)
        qDebug() << "Warning: failed to set PLD_HACC_RAD";

    // Set up MAVLINK passthrough to send rtcm data to drone (no plugin exists for this in MAVSDK v1.2.0)
    mMavlinkPassthrough.reset(new mavsdk::MavlinkPassthrough(mSystem));

}

void MavsdkVehicleConnection::setEnuReference(const llh_t &enuReference)
{
    mEnuReference = enuReference;
}

void MavsdkVehicleConnection::setHomeLlh(const llh_t &homeLlh)
{
    // Set new home in llh via MAVLINK
    mavsdk::MavlinkPassthrough::CommandLong ComLong;
    ComLong.target_compid = mMavlinkPassthrough->get_target_compid();
    ComLong.target_sysid = mMavlinkPassthrough->get_target_sysid();
    ComLong.command = MAV_CMD_DO_SET_HOME;
    ComLong.param1 = 0;
    ComLong.param2 = 0;
    ComLong.param3 = 0;
    ComLong.param4 = NAN;
    ComLong.param5 = homeLlh.latitude;
    ComLong.param6 = homeLlh.longitude;
    ComLong.param7 = homeLlh.height;

    if (mMavlinkPassthrough->send_command_long(ComLong) == mavsdk::MavlinkPassthrough::Result::Success) {
        xyz_t xyz = coordinateTransforms::llhToEnu(mEnuReference, homeLlh);

        auto homePos = mVehicleState->getHomePosition();
        homePos.setX(xyz.x);
        homePos.setY(xyz.y);
        homePos.setHeight(xyz.z);
        mVehicleState->setHomePosition(homePos);
    }
}

void MavsdkVehicleConnection::requestArm()
{
    mAction->arm_async([](mavsdk::Action::Result res){
        if (res != mavsdk::Action::Result::Success)
            qDebug() << "Warning: MavsdkVehicleConnection's arm request failed.";
    });
}

void MavsdkVehicleConnection::requestDisarm()
{
    mAction->disarm_async([](mavsdk::Action::Result res){
        if (res != mavsdk::Action::Result::Success)
            qDebug() << "Warning: MavsdkVehicleConnection's disarm request failed.";
    });
}

void MavsdkVehicleConnection::requestTakeoff()
{
    if (mVehicleType == MAV_TYPE::MAV_TYPE_QUADROTOR) {
        mAction->takeoff_async([](mavsdk::Action::Result res){
            if (res != mavsdk::Action::Result::Success)
                qDebug() << "Warning: MavsdkVehicleConnection's takeoff request failed.";
        });
    } else
        qDebug() << "Warning: MavsdkVehicleConnection is trying to take off with an unknown/incompatible vehicle type, ignored.";
}

void MavsdkVehicleConnection::requestLanding()
{
    if (mVehicleType == MAV_TYPE::MAV_TYPE_QUADROTOR) {
        mAction->land_async([](mavsdk::Action::Result res){
            if (res != mavsdk::Action::Result::Success)
                qDebug() << "Warning: MavsdkVehicleConnection's land request failed.";
        });
    } else
        qDebug() << "Warning: MavsdkVehicleConnection is trying to land with an unknown/incompatible vehicle type, ignored.";
}

void MavsdkVehicleConnection::requestPrecisionLanding()
{
    mavsdk::MavlinkPassthrough::CommandLong ComLong;
    memset(&ComLong, 0, sizeof (ComLong));
    ComLong.target_compid = mMavlinkPassthrough->get_target_compid();
    ComLong.target_sysid = mMavlinkPassthrough->get_target_sysid();
    ComLong.command = MAV_CMD_DO_SET_MODE;
    ComLong.param1 = MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    ComLong.param2 = 4; // PX4_CUSTOM_MAIN_MODE_AUTO
    ComLong.param3 = 9; // PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND

    if (mMavlinkPassthrough->send_command_long(ComLong) != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: MavsdkVehicleConnection's precision land request failed.";
}

void MavsdkVehicleConnection::requestReturnToHome()
{
    if (mVehicleType == MAV_TYPE::MAV_TYPE_QUADROTOR) {
        mAction->return_to_launch_async([](mavsdk::Action::Result res){
            if (res != mavsdk::Action::Result::Success)
                qDebug() << "Warning: MavsdkVehicleConnection's return to home request failed.";
        });
    } else
        qDebug() << "Warning: MavsdkVehicleConnection is trying to land with an unknown/incompatible vehicle type, ignored.";
}

void MavsdkVehicleConnection::requestGotoLlh(const llh_t &llh)
{
    mAction->goto_location_async(llh.latitude, llh.longitude, llh.height, 0, [](mavsdk::Action::Result res){
        if (res != mavsdk::Action::Result::Success)
            qDebug() << "Warning: MavsdkVehicleConnection's goto request failed.";
    });
}

void MavsdkVehicleConnection::requestGotoENU(const xyz_t &xyz)
{
    if (mConvertLocalPositionsToGlobalBeforeSending) {
        llh_t llh = coordinateTransforms::enuToLlh(mEnuReference, xyz);
        requestGotoLlh(llh);
    } else {
        qDebug() << "MavsdkVehicleConnection::requestGotoENU: sending local coordinates to vehicle without converting not implemented.";
    }
}

void MavsdkVehicleConnection::inputRtcmData(const QByteArray &rtcmData)
{
    // See: https://github.com/mavlink/qgroundcontrol/blob/aba881bf8e3f2fdbf63ef0689a3bf0432f597759/src/GPS/RTCM/RTCMMavlink.cc#L24
    if (mMavlinkPassthrough == nullptr)
        return;

    static uint8_t sequenceId = 0;

    mavlink_message_t mavRtcmMsg;
    mavlink_gps_rtcm_data_t mavRtcmData;
    memset(&mavRtcmData, 0, sizeof(mavlink_gps_rtcm_data_t));
    if (rtcmData.length() < MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN) {
        mavRtcmData.len = rtcmData.length();
        mavRtcmData.flags = (sequenceId & 0x1F) << 3;
        memcpy(mavRtcmData.data, rtcmData.data(), rtcmData.size());

        mavlink_msg_gps_rtcm_data_encode(mMavlinkPassthrough->get_target_sysid(), mMavlinkPassthrough->get_target_compid(), &mavRtcmMsg, &mavRtcmData);
        if (mMavlinkPassthrough->send_message(mavRtcmMsg) != mavsdk::MavlinkPassthrough::Result::Success)
            qDebug() << "Warning: could not send RTCM via MAVLINK.";
    } else { // rtcm data needs to be fragmented into multiple messages
        uint8_t fragmentId = 0;
        int numBytesProcessed = 0;
        while (numBytesProcessed < rtcmData.size()) {
            int fragmentLength = std::min(rtcmData.size() - numBytesProcessed, MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN);
            mavRtcmData.flags = 1;                          // LSB set indicates message is fragmented
            mavRtcmData.flags |= fragmentId++ << 1;         // Next 2 bits are fragment id
            mavRtcmData.flags |= (sequenceId & 0x1F) << 3;  // Next 5 bits are sequence id
            mavRtcmData.len = fragmentLength;
            memcpy(mavRtcmData.data, rtcmData.data() + numBytesProcessed, fragmentLength);

            mavlink_msg_gps_rtcm_data_encode(mMavlinkPassthrough->get_target_sysid(), mMavlinkPassthrough->get_target_compid(), &mavRtcmMsg, &mavRtcmData);
            if (mMavlinkPassthrough->send_message(mavRtcmMsg) != mavsdk::MavlinkPassthrough::Result::Success)
                qDebug() << "Warning: could not send RTCM via MAVLINK.";
            numBytesProcessed += fragmentLength;
        }
    }

    sequenceId++;
}

void MavsdkVehicleConnection::sendLandingTargetLlh(const llh_t &landingTargetLlh)
{
    if (mMavlinkPassthrough == nullptr)
        return;

    // Note from https://docs.px4.io/master/en/advanced_features/precland.html#mission
    // The system must publish the coordinates of the target in the LANDING_TARGET message.
    // Note that PX4 requires LANDING_TARGET.frame to be MAV_FRAME_LOCAL_NED and only populates the fields x, y, and z.
    // The origin of the local NED frame [0,0] is the home position.

    // Note by Marvin: not home position, gps origin! I tried to update gps origin using mavlink_set_gps_global_origin_t,
    // but it is not updated while flying (PX4 1.12). Thus, we need to take their gps origin for calculating landing target in NED here.

    // From Llh to their ENU
    xyz_t landingTargetENUgpsOrigin = coordinateTransforms::llhToEnu({mGpsGlobalOrigin.latitude, mGpsGlobalOrigin.longitude, mGpsGlobalOrigin.height}, landingTargetLlh);

    // Send landing target message (with NED frame)
    mavlink_message_t mavLandingTargetMsg;
    mavlink_landing_target_t mavLandingTargetNED;
    memset(&mavLandingTargetNED, 0, sizeof(mavlink_landing_target_t));

    mavLandingTargetNED.position_valid = 1;
    mavLandingTargetNED.frame = MAV_FRAME_LOCAL_NED;
    mavLandingTargetNED.time_usec = QDateTime::currentDateTimeUtc().toMSecsSinceEpoch();

    // ENU -> NED
    mavLandingTargetNED.x = landingTargetENUgpsOrigin.y;
    mavLandingTargetNED.y = landingTargetENUgpsOrigin.x;
    mavLandingTargetNED.z = -landingTargetENUgpsOrigin.z;

    mavlink_msg_landing_target_encode(mMavlinkPassthrough->get_target_sysid(), mMavlinkPassthrough->get_target_compid(), &mavLandingTargetMsg, &mavLandingTargetNED);
    if (mMavlinkPassthrough->send_message(mavLandingTargetMsg) != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: could not send LANDING_TARGET via MAVLINK.";

}

void MavsdkVehicleConnection::sendLandingTargetENU(const xyz_t &landingTargetENU)
{
    if (mMavlinkPassthrough == nullptr)
        return;

    llh_t landingTargetLlh = coordinateTransforms::enuToLlh(mEnuReference, landingTargetENU);
    sendLandingTargetLlh(landingTargetLlh);
}

void MavsdkVehicleConnection::sendSetGpsOriginLlh(const llh_t &gpsOriginLlh)
{
    if (mMavlinkPassthrough == nullptr)
        return;

    mavlink_message_t mavGpsGlobalOriginMsg;
    mavlink_set_gps_global_origin_t mavGpsGlobalOrigin;
    memset(&mavGpsGlobalOrigin, 0, sizeof(mavlink_set_gps_global_origin_t));

    mavGpsGlobalOrigin.latitude = (int) (gpsOriginLlh.latitude * 1e7);
    mavGpsGlobalOrigin.longitude = (int) (gpsOriginLlh.longitude * 1e7);
    mavGpsGlobalOrigin.altitude = (int) (gpsOriginLlh.height * 1e3);
    mavGpsGlobalOrigin.target_system = mMavlinkPassthrough->get_target_sysid();

    mavlink_msg_set_gps_global_origin_encode(mMavlinkPassthrough->get_target_sysid(), mMavlinkPassthrough->get_target_compid(), &mavGpsGlobalOriginMsg, &mavGpsGlobalOrigin);
    if (mMavlinkPassthrough->send_message(mavGpsGlobalOriginMsg) != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: could not send GPS_GLOBAL_ORIGIN via MAVLINK.";
    else
        qDebug() << "Sent GPS_GLOBAL_ORIGIN via MAVLINK:" << gpsOriginLlh.latitude << gpsOriginLlh.longitude;
}

void MavsdkVehicleConnection::setConvertLocalPositionsToGlobalBeforeSending(bool convertLocalPositionsToGlobalBeforeSending)
{
    mConvertLocalPositionsToGlobalBeforeSending = convertLocalPositionsToGlobalBeforeSending;
}

void MavsdkVehicleConnection::setWaypointFollower(const QSharedPointer<WaypointFollower> &waypointFollower)
{
    mWaypointFollower = waypointFollower;
}

bool MavsdkVehicleConnection::hasWaypointFollower()
{
    return !mWaypointFollower.isNull();
}

QSharedPointer<WaypointFollower> MavsdkVehicleConnection::getWaypointFollower() const
{
    return mWaypointFollower;
}
