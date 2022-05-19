#include "cameragimbalui.h"
#include "ui_cameragimbalui.h"

CameraGimbalUI::CameraGimbalUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CameraGimbalUI)
{
    ui->setupUi(this);
    mSetRoiByClickOnMapModule = QSharedPointer<SetRoiByClickOnMapModule>::create(this);
}

CameraGimbalUI::~CameraGimbalUI()
{
    delete ui;
}

void CameraGimbalUI::setGimbal(const QSharedPointer<Gimbal> gimbal)
{
    mGimbal = gimbal;
    if (!gimbal.isNull()) {
        ui->gimbalStatusLabel->setText("Gimbal found.");
        ui->gimbalControlGroup->setEnabled(true);
        ui->cameraControlGroup->setEnabled(true);
    }
}

QSharedPointer<MapModule> CameraGimbalUI::getSetRoiByClickOnMapModule() const
{
    return mSetRoiByClickOnMapModule;
}

void CameraGimbalUI::setVehicleConnection(const QSharedPointer<VehicleConnection> &vehicleConnection)
{
    mVehicleConnection = vehicleConnection;
}

CameraGimbalUI::SetRoiByClickOnMapModule::SetRoiByClickOnMapModule(CameraGimbalUI *parent) : mCameraGimbalUI(parent)
{
    mSetRoiAction = QSharedPointer<QAction>::create(this);
    mRoiContextMenu = QSharedPointer<QMenu>::create();
    mRoiContextMenu->addAction(mSetRoiAction.get());
    connect(mSetRoiAction.get(), &QAction::triggered, [&](){
            mCameraGimbalUI->mGimbal->setRegionOfInterest(mLastClickedMapPos, mLastEnuRefFromMap);
            mLastRoiSet = mLastClickedMapPos;
    });
}

void CameraGimbalUI::SetRoiByClickOnMapModule::processPaint(QPainter &painter, int width, int height, bool highQuality, QTransform drawTrans, QTransform txtTrans, double scale)
{
    Q_UNUSED(width)
    Q_UNUSED(height)
    Q_UNUSED(highQuality)

    if (mLastRoiSet.x != std::numeric_limits<double>::max()) {
        QPen pen;
        pen.setWidthF(5.0 / scale);
        pen.setColor(Qt::darkRed);
        painter.setPen(pen);
        painter.setBrush(Qt::red);
        painter.setTransform(drawTrans);

        QPointF posInWidget(mLastRoiSet.x * 1000.0, mLastRoiSet.y * 1000.0);

        painter.drawEllipse(posInWidget, 10.0 / scale, 10.0 / scale);

        QRectF roiLabelRectangle;
        painter.setTransform(txtTrans);
        posInWidget = drawTrans.map(posInWidget);
        pen.setColor(Qt::black);
        painter.setPen(pen);
        roiLabelRectangle.setCoords(posInWidget.x() + 10, posInWidget.y() - 20,
                                      posInWidget.x() + 50, posInWidget.y() + 50);
        painter.drawText(roiLabelRectangle, Qt::AlignCenter, "ROI");
    }

}

QSharedPointer<QMenu> CameraGimbalUI::SetRoiByClickOnMapModule::populateContextMenu(const xyz_t &mapPos, const llh_t &enuReference)
{
    if (mCameraGimbalUI->mGimbal.isNull())
        return nullptr;

    mSetRoiAction->setText(QString("Set ROI to x=%1, y=%2, z=%3")
                        .arg(mapPos.x)
                        .arg(mapPos.y)
                        .arg(0.0));
    mLastClickedMapPos = mapPos;
    mLastEnuRefFromMap = enuReference;

    return mRoiContextMenu;
}

void CameraGimbalUI::on_actuatorOneLowButton_clicked()
{
    mVehicleConnection->setActuatorOutput(1, -1.0f);
}

void CameraGimbalUI::on_actuatorOneMidButton_clicked()
{
    mVehicleConnection->setActuatorOutput(1, 0.0f);
}

void CameraGimbalUI::on_actuatorOneHighButton_clicked()
{
    mVehicleConnection->setActuatorOutput(1, 1.0f);
}

void CameraGimbalUI::on_actuatorTwoLowButton_clicked()
{
    mVehicleConnection->setActuatorOutput(2, -1.0f);
}

void CameraGimbalUI::on_actuatorTwoMidButton_clicked()
{
    mVehicleConnection->setActuatorOutput(2, 0.0f);
}

void CameraGimbalUI::on_actuatorTwoHighButton_clicked()
{
    mVehicleConnection->setActuatorOutput(2, 1.0f);
}

void CameraGimbalUI::on_zeroButton_clicked()
{
    mPitchYawState = {0.0, 0.0};
    mGimbal->setPitchAndYaw(mPitchYawState.first, mPitchYawState.second);
}

void CameraGimbalUI::on_upButton_clicked()
{
    moveGimbal(SMALL_STEP, 0.0);
}

void CameraGimbalUI::on_rightButton_clicked()
{
    moveGimbal(0.0, SMALL_STEP);
}

void CameraGimbalUI::on_downButton_clicked()
{
    moveGimbal(-SMALL_STEP, 0.0);
}

void CameraGimbalUI::on_leftButton_clicked()
{
    moveGimbal(0.0, -SMALL_STEP);
}

void CameraGimbalUI::on_doubleUpButton_clicked()
{
    moveGimbal(MEDIUM_STEP, 0.0);
}

void CameraGimbalUI::on_doubleRightButton_clicked()
{
    moveGimbal(0.0, MEDIUM_STEP);
}

void CameraGimbalUI::on_doubleDownButton_clicked()
{
    moveGimbal(-MEDIUM_STEP, 0.0);
}

void CameraGimbalUI::on_doubleLeftButton_clicked()
{
    moveGimbal(0.0, -MEDIUM_STEP);
}

void CameraGimbalUI::on_tripleUpButton_clicked()
{
    moveGimbal(BIG_STEP, 0.0);
}

void CameraGimbalUI::on_tripleRightButton_clicked()
{
    moveGimbal(0.0, BIG_STEP);
}

void CameraGimbalUI::on_tripleDownButton_clicked()
{
    moveGimbal(-BIG_STEP, 0.0);
}

void CameraGimbalUI::on_trippleLeftButton_clicked()
{
    moveGimbal(0.0, -BIG_STEP);
}

void CameraGimbalUI::moveGimbal(double pitch_deg, double yaw_deg)
{
    mPitchYawState.first += pitch_deg;
    mPitchYawState.second += yaw_deg;

    // ignore pitch requests outside of range
    if (mPitchYawState.first > PITCH_RANGE.second)
        mPitchYawState.first = PITCH_RANGE.second;
    if (mPitchYawState.first < PITCH_RANGE.first)
        mPitchYawState.first = PITCH_RANGE.first;

    // normalize yaw requests
    while (mPitchYawState.second > YAW_RANGE.second)
        mPitchYawState.second -= 360.0;
    while (mPitchYawState.second < YAW_RANGE.first)
        mPitchYawState.second += 360.0;

    mGimbal->setPitchAndYaw(mPitchYawState.first, mPitchYawState.second);
}

void CameraGimbalUI::on_yawFollowButton_clicked()
{
    mGimbal->setYawLocked(false);
}

void CameraGimbalUI::on_yawLockButton_clicked()
{
    mGimbal->setYawLocked(true);
}
