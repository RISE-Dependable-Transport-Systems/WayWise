#include <QCoreApplication>
#include "../../core/simplewatchdog.h"
#include "../../vehicles/carstate.h"
#include "../../vehicles/controller/carmovementcontroller.h"
#include "../../autopilot/purepursuitwaypointfollower.h"
#include "../../communication/mavsdkvehicleserver.h"
#include "../../logger/logger.h"

int main(int argc, char *argv[])
{
    Logger::initVehicle();

    QCoreApplication a(argc, argv);
    const int mUpdateVehicleStatePeriod_ms = 25;
    QTimer mUpdateVehicleStateTimer;

    QSharedPointer<CarState> mCarState(new CarState);
    MavsdkVehicleServer mavsdkVehicleServer(mCarState);

    // --- Lower-level control setup ---
    QSharedPointer<CarMovementController> mCarMovementController(new CarMovementController(mCarState));

    QObject::connect(&mUpdateVehicleStateTimer, &QTimer::timeout, [&](){
        mCarState->simulationStep(mUpdateVehicleStatePeriod_ms, PosType::fused);
    });
    mUpdateVehicleStateTimer.start(mUpdateVehicleStatePeriod_ms);

    // --- Autopilot ---
    QSharedPointer<PurepursuitWaypointFollower> mWaypointFollower(new PurepursuitWaypointFollower(mCarMovementController));
    mWaypointFollower->setPurePursuitRadius(1.0);
    mWaypointFollower->setRepeatRoute(false);

    // Setup MAVLINK communication towards ControlTower
    mavsdkVehicleServer.setMovementController(mCarMovementController);
    mavsdkVehicleServer.setWaypointFollower(mWaypointFollower);

    // Watchdog that warns when EventLoop is slowed down
    SimpleWatchdog watchdog;

    qDebug() << "\n" // by hjw
             << "                    .------.\n"
             << "                    :|||\"\"\"`.`.\n"
             << "                    :|||     7.`.\n"
             << " .===+===+===+===+===||`----L7'-`7`---.._\n"
             << " [] MAVLINK autopilot|| ==       |       \"\"\"-.\n"
             << " []...._____.example.||........../ _____ ____|\n"
             << "c\\____/,---.\\_       ||_________/ /,---.\\_  _/\n"
             << "  /_,-/ ,-. \\ `._____|__________||/ ,-. \\ \\_[\n"
             << "     /\\ `-' /                    /\\ `-' /\n"
             << "       `---'                       `---'\n";

    return a.exec();
}
