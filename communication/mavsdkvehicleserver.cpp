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

    // -- NOTE #1: next MAVSDK release (1.5?) should simplify the following code to this:
    // auto server_component = mMavsdk.server_component_by_type(Mavsdk::ServerComponentType::Autopilot);
    auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
    auto fut = prom.get_future();
    mMavsdk.subscribe_on_new_system([this, &prom]() {
        qDebug() << "Discovered MAVSDK GCS";
        auto system = mMavsdk.systems().back();
        mMavsdk.subscribe_on_new_system(nullptr);
        prom.set_value(system);
    });

    qDebug() << "Sleeping AP thread... ";
    for (auto i = 0; i < 3; i++) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        if (mMavsdk.systems().size() == 0) {
            qDebug() << "No System Found from Autopilot, trying again in 5 secs...";
            if (i == 2) {
                qDebug() << "No System found after three retries. Aborting...";
                return;
            }
        } else {
            qDebug() << "Setting System";
            break;
        }
    }
    auto system = mMavsdk.systems().back();

    system->set_vendor_id(12321); // TODO: this happens too late (GCS has already read info)
   // NOTE #1 --


    // Create server plugins
    mTelemetryServer.reset(new mavsdk::TelemetryServer(system));
    mActionServer.reset(new mavsdk::ActionServer(system));
    mParamServer.reset(new mavsdk::ParamServer(system));
    mMissionRawServer.reset(new mavsdk::MissionRawServer(system));

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

        mavsdk::TelemetryServer::Heading heading{mVehicleState->getPosition(PosType::fused).getYaw() + 90.0}; // heading NED

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

    mActionServer->subscribe_flight_mode_change([this](mavsdk::ActionServer::Result res, mavsdk::ActionServer::FlightMode mode){
       if  (res == mavsdk::ActionServer::Result::Success)
           switch (mode) {
           case mavsdk::ActionServer::FlightMode::Mission:
               emit startWaypointFollower(false);
               break;
           default:
               if (mWaypointFollower->isActive())
                   emit pauseWaypointFollower();
           }

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

    mMissionRawServer->subscribe_current_item_changed([this](mavsdk::MissionRawServer::MissionItem item){
        if (!mWaypointFollower.isNull() && item.seq == 0)
            emit resetWaypointFollower();
        else if (item.seq != 0)
            qDebug() << "Warning: jumping to seq ID in mission not implemented in MavsdkVehicleServer / WaypointFollower.";

    });
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
}

PosPoint MavsdkVehicleServer::convertMissionItemToPosPoint(const mavsdk::MissionRawServer::MissionItem &item)
{
    PosPoint routePoint;

    routePoint.setX(item.x / 10e4);
    routePoint.setY(item.y / 10e4);
    routePoint.setHeight(item.z);

//    qDebug() << routePoint.getX() << routePoint.getY() << routePoint.getHeight();

    return routePoint;
}
