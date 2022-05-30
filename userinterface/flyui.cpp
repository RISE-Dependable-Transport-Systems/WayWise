/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "flyui.h"
#include "ui_flyui.h"

FlyUI::FlyUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FlyUI)
{
    ui->setupUi(this);
    mGotoClickOnMapModule = QSharedPointer<FlyUI::GotoClickOnMapModule>::create(this);
}

FlyUI::~FlyUI()
{
    delete ui;
}

void FlyUI::setCurrentVehicleConnection(const QSharedPointer<MavsdkVehicleConnection> &currentVehicleConnection)
{
    mCurrentVehicleConnection = currentVehicleConnection;
}

QSharedPointer<MapModule> FlyUI::getGotoClickOnMapModule()
{
    return mGotoClickOnMapModule;
}

void FlyUI::on_takeoffButton_clicked()
{
    if (mCurrentVehicleConnection) {
        mCurrentVehicleConnection->requestTakeoff();
    }
}

void FlyUI::on_armButton_clicked()
{
    if (mCurrentVehicleConnection) {
        mCurrentVehicleConnection->requestArm();
    }
}

void FlyUI::on_disarmButton_clicked()
{
    if (mCurrentVehicleConnection) {
        mCurrentVehicleConnection->requestDisarm();
    }
}

void FlyUI::on_returnToHomeButton_clicked()
{
    if (mCurrentVehicleConnection) {
        mCurrentVehicleConnection->requestReturnToHome();
    }
}

void FlyUI::on_landButton_clicked()
{
    if (mCurrentVehicleConnection) {
        mCurrentVehicleConnection->requestLanding();
    }
}

void FlyUI::on_gotoButton_clicked()
{
    if (mCurrentVehicleConnection) {
        mCurrentVehicleConnection->requestGotoENU({ui->gotoXSpinBox->value(), ui->gotoYSpinBox->value(), ui->gotoZSpinBox->value()});
    }
}

void FlyUI::on_getPositionButton_clicked()
{
    if (mCurrentVehicleConnection) {
        ui->gotoXSpinBox->setValue(mCurrentVehicleConnection->getVehicleState()->getPosition().getX());
        ui->gotoYSpinBox->setValue(mCurrentVehicleConnection->getVehicleState()->getPosition().getY());
        ui->gotoZSpinBox->setValue(mCurrentVehicleConnection->getVehicleState()->getPosition().getHeight());
    }
}

void FlyUI::on_apRestartButton_clicked()
{
    if (mCurrentVehicleConnection->hasWaypointFollower())
        mCurrentVehicleConnection->getWaypointFollower()->startFollowingRoute(true);
}

void FlyUI::on_apStartButton_clicked()
{
    if (mCurrentVehicleConnection->hasWaypointFollower())
        mCurrentVehicleConnection->getWaypointFollower()->startFollowingRoute(false);
}

void FlyUI::on_apPauseButton_clicked()
{
    if (mCurrentVehicleConnection->hasWaypointFollower())
        mCurrentVehicleConnection->getWaypointFollower()->stop();
}

void FlyUI::on_apStopButton_clicked()
{
    if (mCurrentVehicleConnection->hasWaypointFollower()) {
        mCurrentVehicleConnection->getWaypointFollower()->stop();
        mCurrentVehicleConnection->getWaypointFollower()->resetState();
    }
}

QSharedPointer<MavsdkVehicleConnection> FlyUI::getCurrentVehicleConnection() const
{
    return mCurrentVehicleConnection;
}

void FlyUI::gotRouteForAutopilot(const QList<PosPoint> &route)
{
    if (!mCurrentVehicleConnection->hasWaypointFollower()) {
        mCurrentVehicleConnection->setWaypointFollower(QSharedPointer<WaypointFollower>::create(mCurrentVehicleConnection, PosType::defaultPosType));
        mCurrentVehicleConnection->getWaypointFollower()->setPurePursuitRadius(3.0);
    }

    mCurrentVehicleConnection->getWaypointFollower()->clearRoute();
    mCurrentVehicleConnection->getWaypointFollower()->addRoute(route);
}



FlyUI::GotoClickOnMapModule::GotoClickOnMapModule(FlyUI *parent) : mFlyUI(parent)
{
    mGotoAction = QSharedPointer<QAction>::create(this);
    mGotoContextMenu = QSharedPointer<QMenu>::create();
    mGotoContextMenu->addAction(mGotoAction.get());
    connect(mGotoAction.get(), &QAction::triggered, [&](){
        if (mFlyUI->mCurrentVehicleConnection) {
            if (mFlyUI->mCurrentVehicleConnection->hasWaypointFollower() && mFlyUI->mCurrentVehicleConnection->getWaypointFollower()->isActive()) {
                mFlyUI->mCurrentVehicleConnection->getWaypointFollower()->stop();
                qDebug() << "Note: Waypointfollower stopped by goto request from map.";
            }

            mFlyUI->mCurrentVehicleConnection->requestGotoENU({mLastClickedMapPos.x, mLastClickedMapPos.y,
                                                               mFlyUI->mCurrentVehicleConnection->getVehicleState()->getPosition().getHeight()}, true);
        }
    });
}

void FlyUI::GotoClickOnMapModule::processPaint(QPainter &painter, int width, int height, bool highQuality, QTransform drawTrans, QTransform txtTrans, double scale)
{
    Q_UNUSED(painter)
    Q_UNUSED(width)
    Q_UNUSED(height)
    Q_UNUSED(highQuality)
    Q_UNUSED(drawTrans)
    Q_UNUSED(txtTrans)
    Q_UNUSED(scale)
}

void FlyUI::on_precisionLandButton_clicked()
{
    if (mCurrentVehicleConnection) {
        mCurrentVehicleConnection->requestPrecisionLanding();
    }
}


QSharedPointer<QMenu> FlyUI::GotoClickOnMapModule::populateContextMenu(const xyz_t &mapPos, const llh_t &enuReference)
{
    Q_UNUSED(enuReference)
    if (mFlyUI->mCurrentVehicleConnection.isNull())
        return nullptr;
    else {
        mGotoAction->setText(QString("Goto x=%1, y=%2, z=%3")
                            .arg(mapPos.x)
                            .arg(mapPos.y)
                            .arg(mFlyUI->mCurrentVehicleConnection->getVehicleState()->getPosition().getHeight()));
        mLastClickedMapPos = mapPos;
        return mGotoContextMenu;
    }
}
