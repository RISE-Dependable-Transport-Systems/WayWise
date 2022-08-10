/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
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

void PlanUI::on_heightSpinBox_valueChanged(double arg1)
{
    mRoutePlanner->setNewPointHeight(arg1);
}

void PlanUI::on_speedSpinBox_valueChanged(double arg1)
{
    mRoutePlanner->setNewPointSpeed(arg1 / 3.6);
}

void PlanUI::on_attributeLineEdit_textChanged(const QString &arg1)
{
    QString tmp = arg1;
    mRoutePlanner->setNewPointAttribute(tmp.replace(" ", "").toUInt(nullptr, 16));
}

void PlanUI::on_updatePointCheckBox_toggled(bool checked)
{
    mRoutePlanner->setUpdatePointOnClick(checked);
}
