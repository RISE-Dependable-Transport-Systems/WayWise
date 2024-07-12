/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *               2024 Rickard Häll      rickard.hall@ri.se
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

void FlyUI::setCurrentVehicleConnection(const QSharedPointer<VehicleConnection> &currentVehicleConnection)
{
    mCurrentVehicleConnection = currentVehicleConnection;

    if (mCurrentVehicleConnection)
        if (!mCurrentVehicleConnection->hasFollowPointConnectionLocal())
            mCurrentVehicleConnection->setFollowPointConnectionLocal(QSharedPointer<FollowPoint>::create(mCurrentVehicleConnection, PosType::defaultPosType));

    connect(ui->followVehicleIdCombo, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &FlyUI::updateVehicleIdToFollow);
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
    if (mCurrentVehicleConnection.isNull())
        return;

    if (ui->apExecuteRouteRadioButton->isChecked())
        mCurrentVehicleConnection->restartAutopilot();
    else
        mCurrentVehicleConnection->startFollowPoint();
}

void FlyUI::on_apStartButton_clicked()
{
    if (mCurrentVehicleConnection.isNull())
        return;

    if (ui->apExecuteRouteRadioButton->isChecked())
        mCurrentVehicleConnection->startAutopilot();
    else
        mCurrentVehicleConnection->startFollowPoint();
}

void FlyUI::on_apPauseButton_clicked()
{
    if (mCurrentVehicleConnection) {
        if (ui->apExecuteRouteRadioButton->isChecked())
            mCurrentVehicleConnection->pauseAutopilot();
        else
            mCurrentVehicleConnection->stopFollowPoint();
    }
}

void FlyUI::on_apStopButton_clicked()
{
    if (mCurrentVehicleConnection) {
        if (ui->apExecuteRouteRadioButton->isChecked())
            mCurrentVehicleConnection->stopAutopilot();
        else
            mCurrentVehicleConnection->stopFollowPoint();
    }
}

QSharedPointer<VehicleConnection> FlyUI::getCurrentVehicleConnection() const
{
    return mCurrentVehicleConnection;
}

void FlyUI::gotRouteForAutopilot(const QList<PosPoint> &route)
{
    if (mCurrentVehicleConnection.isNull())
        return;

    if (!mCurrentVehicleConnection->hasWaypointFollowerConnectionLocal())
        mCurrentVehicleConnection->setWaypointFollowerConnectionLocal(QSharedPointer<GotoWaypointFollower>::create(mCurrentVehicleConnection, PosType::defaultPosType));

    bool safetyCheckFailed = false;
    for (int i=0; i<route.length(); i++) {
        if (mCurrentVehicleConnection->getVehicleState()->getHomePosition().getDistanceTo3d(route.at(i)) > mLineOfSightDistance){
            qDebug() << "Warning: Waypoint:" << i << "is beyond line of sight of" << mLineOfSightDistance << "m. Route is discarded.";
            safetyCheckFailed = true;
            break;
        }
    }

    if (safetyCheckFailed)
        mCurrentVehicleConnection->clearRoute();
    else
        mCurrentVehicleConnection->setRoute(route);
}



FlyUI::GotoClickOnMapModule::GotoClickOnMapModule(FlyUI *parent) : mFlyUI(parent)
{
    mGotoAction = QSharedPointer<QAction>::create(this);
    mGotoContextMenu = QSharedPointer<QMenu>::create();
    mGotoContextMenu->addAction(mGotoAction.get());
    connect(mGotoAction.get(), &QAction::triggered, [&](){
        if (mFlyUI->mCurrentVehicleConnection) {
            if (mFlyUI->mCurrentVehicleConnection->isAutopilotActive()) {
                mFlyUI->mCurrentVehicleConnection->stopAutopilot();
                qDebug() << "Note: Autopilot stopped by goto request from map.";
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

void FlyUI::on_vehicleParameterButton_clicked()
{
    if (mVehicleParameterUI.isNull())
        mVehicleParameterUI = QSharedPointer<VehicleParameterUI>::create(this);
    mVehicleParameterUI->setCurrentVehicleConnection(mCurrentVehicleConnection);
    mVehicleParameterUI->show();
    this->releaseKeyboard();
}

void FlyUI::on_pollENUrefButton_clicked()
{
    if (mCurrentVehicleConnection)
        mCurrentVehicleConnection->pollCurrentENUreference();
}

void FlyUI::updateFollowVehicleIdComboBox(const QList<QSharedPointer<MavsdkVehicleConnection>> &vehicleConnectionList)
{
    int previousCurrentIndex = ui->followVehicleIdCombo->currentIndex() < 0 ? 0 : ui->followVehicleIdCombo->currentIndex();

    ui->followVehicleIdCombo->clear();
    for (const auto& vehicleConnection : vehicleConnectionList) {
        if (mCurrentVehicleConnection)
            if(mCurrentVehicleConnection->getVehicleState()->getId() != vehicleConnection->getVehicleState()->getId())
                ui->followVehicleIdCombo->addItem(QString::number(vehicleConnection->getVehicleState()->getId()),
                                                  QVariant::fromValue(vehicleConnection));
    }
    ui->followVehicleIdCombo->setCurrentIndex((previousCurrentIndex < ui->followVehicleIdCombo->count()) ? previousCurrentIndex : (ui->followVehicleIdCombo->count()-1));
}

void FlyUI::updateVehicleIdToFollow(int vehicleId)
{
    if (mCurrentVehicleConnection.isNull())
        return;

    static int previousVehicleId = 0;
    disconnect(ui->followVehicleIdCombo->itemData(previousVehicleId).value<QSharedPointer<MavsdkVehicleConnection>>()->getVehicleState().get(), &VehicleState::positionUpdated,  this, nullptr);

    if (ui->followVehicleIdCombo->count() == 0)
        return;

    QSharedPointer<VehicleState> vehicleState = ui->followVehicleIdCombo->itemData(vehicleId).value<QSharedPointer<MavsdkVehicleConnection>>()->getVehicleState();

    connect(vehicleState.get(), &ObjectState::positionUpdated, this, [this, vehicleState](){
        auto positionOfVehicleToFollow = vehicleState->getPosition();
        positionOfVehicleToFollow.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
        mCurrentVehicleConnection->updatePointToFollowInEnuFrame(positionOfVehicleToFollow);
    }, Qt::QueuedConnection);

    previousVehicleId = vehicleId;
}
