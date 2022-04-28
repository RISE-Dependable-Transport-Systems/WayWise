#include "planui.h"
#include "ui_planui.h"

PlanUI::PlanUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PlanUI)
{
    ui->setupUi(this);


    mRoutePlanner = QSharedPointer<RoutePlannerModule>::create();
}

PlanUI::~PlanUI()
{
    delete ui;
}

QSharedPointer<RoutePlannerModule> PlanUI::getRoutePlannerModule() const
{
    return mRoutePlanner;
}

void PlanUI::on_addRouteButton_clicked()
{
    mRoutePlanner->addNewRoute();
    ui->currentRouteSpinBox->setValue(mRoutePlanner->getCurrentRouteIndex() + 1);
    ui->currentRouteSpinBox->setMaximum(mRoutePlanner->getNumberOfRoutes());
    ui->currentRouteSpinBox->setSuffix(" / " + QString::number(mRoutePlanner->getNumberOfRoutes()));
}

void PlanUI::on_removeRouteButton_clicked()
{
    mRoutePlanner->removeCurrentRoute();
    ui->currentRouteSpinBox->setValue(mRoutePlanner->getCurrentRouteIndex() + 1);
    ui->currentRouteSpinBox->setMaximum(mRoutePlanner->getNumberOfRoutes());
    ui->currentRouteSpinBox->setSuffix(" / " + QString::number(mRoutePlanner->getNumberOfRoutes()));
}

void PlanUI::on_currentRouteSpinBox_valueChanged(int value)
{
    mRoutePlanner->setCurrentRouteIndex(value-1);
}

void PlanUI::on_sendToAutopilotButton_clicked()
{
    emit routeDoneForUse(mRoutePlanner->getCurrentRoute());
}
