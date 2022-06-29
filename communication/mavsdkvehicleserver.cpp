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
   // NOTE #1 --

    // Create server plugins
    mTelemetryServer.reset(new mavsdk::TelemetryServer(system));
    mActionServer.reset(new mavsdk::ActionServer(system));
    mParamServer.reset(new mavsdk::ParamServer(system));

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

        mavsdk::TelemetryServer::Heading heading{mVehicleState->getPosition(PosType::fused).getYaw()};

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
}

void MavsdkVehicleServer::setUbloxRover(QSharedPointer<UbloxRover> ubloxRover)
{
    mUbloxRover = ubloxRover;
}
