#include "traceui.h"
#include "ui_traceui.h"

TraceUI::TraceUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TraceUI)
{
    ui->setupUi(this);
    mTracemodule = QSharedPointer<TraceModule>::create();
}

TraceUI::~TraceUI()
{
    delete ui;
}

void TraceUI::setCurrentTraceVehicle(QSharedPointer<VehicleState> vehicle)
{
    mTracemodule->setCurrentTraceVehicle(vehicle);
}

QSharedPointer<TraceModule> TraceUI::getTraceModule() const
{
    return mTracemodule;
}


void TraceUI::on_startTraceButton_clicked()
{
    mTracemodule->startTrace(0);
}

void TraceUI::on_stopTraceButton_clicked()
{
    mTracemodule->stopTrace();
}

void TraceUI::on_clearTraceButton_clicked()
{
    mTracemodule->clearTraceIndex(mTracemodule->getCurrentTraceIndex());
}
