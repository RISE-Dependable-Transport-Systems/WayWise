#include <QCoreApplication>
#include "core/simplewatchdog.h"
#include "vehicles/carstate.h"
#include "vehicles/controller/carmovementcontroller.h"
#include "autopilot/purepursuitwaypointfollower.h"
#include "communication/parameterserver.h"
#include "communication/iso22133vehicleserver.h"
#include "logger/logger.h"

int main(int argc, char *argv[])
{
    Logger::initVehicle();

    QCoreApplication a(argc, argv);
    const int mUpdateVehicleStatePeriod_ms = 25;
    QTimer mUpdateVehicleStateTimer;

    QSharedPointer<CarState> mCarState(new CarState);
    iso22133VehicleServer iso22133VehicleServer(mCarState, "127.0.0.1");

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

    // Setup ISO22133 communication towards ATOS
    iso22133VehicleServer.setMovementController(mCarMovementController);
    iso22133VehicleServer.setWaypointFollower(mWaypointFollower);

    // Watchdog that warns when EventLoop is slowed down
    SimpleWatchdog watchdog;

    qDebug() << "\n" // by hjw
             << "                    .------.\n"
             << "                    :|||\"\"\"`.`.\n"
             << "                    :|||     7.`.\n"
             << " .===+===+===+===+===||`----L7'-`7`---.._\n"
             << " []ISO22133 autopilot|| ==       |       \"\"\"-.\n"
             << " []...._____.example.||........../ _____ ____|\n"
             << "c\\____/,---.\\_       ||_________/ /,---.\\_  _/\n"
             << "  /_,-/ ,-. \\ `._____|__________||/ ,-. \\ \\_[\n"
             << "     /\\ `-' /                    /\\ `-' /\n"
             << "       `---'                       `---'\n";

    return a.exec();
}
