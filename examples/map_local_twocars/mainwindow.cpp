#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->mapWidget->setScaleFactor(0.04);

    // Prepare Car 1
    car1 = QSharedPointer<CarState>::create(1, Qt::blue);
    ui->mapWidget->addObjectState(car1);
    car1->setSpeed(2);
    car1->setSteering(0.5);

    mCarMovementController1 = QSharedPointer<CarMovementController>::create(car1);
    QObject::connect(mCarMovementController1.get(), &CarMovementController::updatedOdomPositionAndYaw, [&](QSharedPointer<VehicleState> vehicleState, double distanceDriven){
        Q_UNUSED(distanceDriven)
        PosPoint currentPosition = vehicleState->getPosition(PosType::odom);
        currentPosition.setType(PosType::fused);
        vehicleState->setPosition(currentPosition);
    });

    // Prepare Car 2
    car2 = QSharedPointer<CarState>::create(2, Qt::red);
    ui->mapWidget->addObjectState(car2);
    car2->setSpeed(2);
    car2->setSteering(-0.5);

    mCarMovementController2 = QSharedPointer<CarMovementController>::create(car2);
    QObject::connect(mCarMovementController2.get(), &CarMovementController::updatedOdomPositionAndYaw, [&](QSharedPointer<VehicleState> vehicleState, double distanceDriven){
        Q_UNUSED(distanceDriven)
        PosPoint currentPosition = vehicleState->getPosition(PosType::odom);
        currentPosition.setType(PosType::fused);
        vehicleState->setPosition(currentPosition);
    });

    // Setup simulation
    QObject::connect(&mUpdateVehicleStateTimer, &QTimer::timeout, [=](){
        mCarMovementController1->simulationStep(mUpdateVehicleStatePeriod_ms);
        mCarMovementController2->simulationStep(mUpdateVehicleStatePeriod_ms);
    });
    mUpdateVehicleStateTimer.start(mUpdateVehicleStatePeriod_ms);
}

MainWindow::~MainWindow()
{
    delete ui;
}

