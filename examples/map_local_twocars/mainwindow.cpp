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

    // Prepare Car 2
    car2 = QSharedPointer<CarState>::create(2, Qt::red);
    ui->mapWidget->addObjectState(car2);
    car2->setSpeed(2);
    car2->setSteering(-0.5);

    // Setup simulation
    QObject::connect(&mUpdateVehicleStateTimer, &QTimer::timeout, [=](){
        car1->simulationStep(mUpdateVehicleStatePeriod_ms);
        car2->simulationStep(mUpdateVehicleStatePeriod_ms);
    });
    mUpdateVehicleStateTimer.start(mUpdateVehicleStatePeriod_ms);
}

MainWindow::~MainWindow()
{
    delete ui;
}

