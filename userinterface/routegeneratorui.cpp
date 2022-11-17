#include "routegeneratorui.h"
#include "ui_routegeneratorui.h"
#include "routeplanning/zigzagroutegenerator.h"

RouteGeneratorUI::RouteGeneratorUI(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RouteGeneratorUI)
{
    ui->setupUi(this);
    ui->generatorsTabWidget->tabBar()->setStyle(new RouteGeneratorUI::CustomTabStyle);
    ui->useRouteButton->setIcon(QApplication::style()->standardIcon(QStyle::SP_DialogOkButton));
    ui->cancelButton->setIcon(QApplication::style()->standardIcon(QStyle::SP_DialogCancelButton));

    mRoutePlannerModule = QSharedPointer<RoutePlannerModule>::create();
    mRoutePlannerModule->addNewRoute(); // 0 = bound, 1 = generated route
    mRoutePlannerModule->setDrawRouteText(false);
    ui->previewMapWidget->addMapModule(mRoutePlannerModule);

    ui->boundDoneButton->setEnabled(false);
    ui->page_2->setEnabled(false);
    connect(mRoutePlannerModule.get(), &RoutePlannerModule::requestRepaint, [this](){
        bool boundReady = mRoutePlannerModule->getRoute(boundRouteIndex).size() > 2;
        ui->boundDoneButton->setEnabled(boundReady);
        ui->page_2->setEnabled(boundReady);
    });
}

RouteGeneratorUI::~RouteGeneratorUI()
{
    delete ui;
}

void RouteGeneratorUI::setEnuRef(const llh_t &llh)
{
    ui->previewMapWidget->setEnuRef(llh);
}

void RouteGeneratorUI::on_zigZagToolbox_currentChanged(int index)
{
    if (index <= 1)
        mRoutePlannerModule->setCurrentRouteIndex(index);
}

void RouteGeneratorUI::on_resetBoundButton_clicked()
{
    mRoutePlannerModule->clearCurrentRoute();
}

void RouteGeneratorUI::on_boundDoneButton_clicked()
{
    ui->zigZagToolbox->setCurrentIndex(1);
    updatePreviewRoute();
}

void RouteGeneratorUI::updatePreviewRoute()
{
    QList<PosPoint> bounds = mRoutePlannerModule->getRoute(boundRouteIndex);

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

void RouteGeneratorUI::on_speedStraightsSpinBox_valueChanged(double arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorUI::on_speedTurnsSpinBox_valueChanged(double arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorUI::on_rowSpacingSpinBox_valueChanged(double arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorUI::on_visitEverySpinBox_valueChanged(int arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorUI::on_attributeStraightEdit_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorUI::on_attributeTurnEdit_textEdited(const QString &arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorUI::on_attributeDistanceAfterTurnSpinBox_valueChanged(double arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorUI::on_attributeDistanceBeforeTurnSpinBox_valueChanged(double arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorUI::on_stepsForTurningSpinBox_valueChanged(int arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorUI::on_forceTurnsIntoBoundsCheckBox_stateChanged(int arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorUI::on_generateFrameCheckBox_stateChanged(int arg1)
{
    Q_UNUSED(arg1)
    updatePreviewRoute();
}


void RouteGeneratorUI::on_cancelButton_clicked()
{
    hide();
}


void RouteGeneratorUI::on_useRouteButton_clicked()
{
    if (!mRoutePlannerModule->getRoute(generatedRouteIndex).isEmpty())
        emit routeDoneForUse(mRoutePlannerModule->getRoute(generatedRouteIndex));
    hide();
}

