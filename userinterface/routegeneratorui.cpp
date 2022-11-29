/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "routegeneratorui.h"
#include "ui_routegeneratorui.h"
#include "userinterface/routegeneratorzigzagui.h"

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

    // Instantiate the available generators (only ZigZag currently)
    ui->generatorsTabWidget->addTab(new RouteGeneratorZigZagUI(mRoutePlannerModule), "ZigZag");
}

RouteGeneratorUI::~RouteGeneratorUI()
{
    delete ui;
}

void RouteGeneratorUI::setEnuRef(const llh_t &llh)
{
    ui->previewMapWidget->setEnuRef(llh);
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

