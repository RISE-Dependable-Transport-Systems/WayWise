/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "routegeneratorzigzagui.h"
#include "ui_routegeneratorzigzagui.h"

RouteGeneratorZigZagUI::RouteGeneratorZigZagUI(QSharedPointer<RoutePlannerModule> routePlannerModule, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RouteGeneratorZigZagUI)
{
    ui->setupUi(this);
    mRoutePlannerModule = routePlannerModule;

    ui->boundDoneButton->setEnabled(false);
    ui->page_2->setEnabled(false);
    connect(mRoutePlannerModule.get(), &RoutePlannerModule::requestRepaint, [this](){
        bool boundReady = mRoutePlannerModule->getRoute(RouteGeneratorUI::boundRouteIndex).size() > 2;
        ui->boundDoneButton->setEnabled(boundReady);
        ui->page_2->setEnabled(boundReady);
    });
}

RouteGeneratorZigZagUI::~RouteGeneratorZigZagUI()
{
    delete ui;
}

void RouteGeneratorZigZagUI::on_zigZagToolbox_currentChanged(int index)
{
    switch (index) {
    case 0:
        mRoutePlannerModule->setCurrentRouteIndex(RouteGeneratorUI::boundRouteIndex);
        break;
    case 1:
        mRoutePlannerModule->setCurrentRouteIndex(RouteGeneratorUI::generatedRouteIndex);
        break;
    }
}

void RouteGeneratorZigZagUI::on_resetBoundButton_clicked()
{
    mRoutePlannerModule->clearCurrentRoute();
}

void RouteGeneratorZigZagUI::on_boundDoneButton_clicked()
{
    ui->zigZagToolbox->setCurrentIndex(1);
    updatePreviewRoute();
}

void RouteGeneratorZigZagUI::updatePreviewRoute()
{
    QList<PosPoint> bounds = mRoutePlannerModule->getRoute(RouteGeneratorUI::boundRouteIndex);

    QList<PosPoint> route;
    if (ui->generateFrameCheckBox->isChecked())
        route = ZigZagRouteGenerator::fillConvexPolygonWithFramedZigZag(bounds, ui->rowSpacingSpinBox->value(), ui->forceTurnsIntoBoundsCheckBox->isChecked(), ui->speedStraightsSpinBox->value()/3.6,
                                                              ui->speedTurnsSpinBox->value()/3.6, ui->stepsForTurningSpinBox->value(), ui->visitEverySpinBox->value(),
                                                              ui->attributeStraightEdit->text().replace(" ", "").toUInt(nullptr, 16), ui->attributeTurnEdit->text().replace(" ", "").toUInt(nullptr, 16),
                                                              ui->attributeDistanceAfterTurnSpinBox->value()*2, ui->attributeDistanceBeforeTurnSpinBox->value()*2);
                                                              // attribute changes at half distance
    else
        route = ZigZagRouteGenerator::fillConvexPolygonWithZigZag(bounds, ui->rowSpacingSpinBox->value(), ui->forceTurnsIntoBoundsCheckBox->isChecked(), ui->speedStraightsSpinBox->value()/3.6,
                                                                  ui->speedTurnsSpinBox->value()/3.6, ui->stepsForTurningSpinBox->value(), ui->visitEverySpinBox->value(),
                                                                  ui->attributeStraightEdit->text().replace(" ", "").toUInt(nullptr, 16), ui->attributeTurnEdit->text().replace(" ", "").toUInt(nullptr, 16),
                                                                  ui->attributeDistanceAfterTurnSpinBox->value()*2, ui->attributeDistanceBeforeTurnSpinBox->value()*2);

    mRoutePlannerModule->clearCurrentRoute();
    mRoutePlannerModule->appendRouteToCurrentRoute(route);
}

void RouteGeneratorZigZagUI::on_speedStraightsSpinBox_valueChanged(double arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorZigZagUI::on_speedTurnsSpinBox_valueChanged(double arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorZigZagUI::on_rowSpacingSpinBox_valueChanged(double arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorZigZagUI::on_visitEverySpinBox_valueChanged(int arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorZigZagUI::on_attributeStraightEdit_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorZigZagUI::on_attributeTurnEdit_textEdited(const QString &arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorZigZagUI::on_attributeDistanceAfterTurnSpinBox_valueChanged(double arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorZigZagUI::on_attributeDistanceBeforeTurnSpinBox_valueChanged(double arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorZigZagUI::on_stepsForTurningSpinBox_valueChanged(int arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorZigZagUI::on_forceTurnsIntoBoundsCheckBox_stateChanged(int arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorZigZagUI::on_generateFrameCheckBox_stateChanged(int arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}
