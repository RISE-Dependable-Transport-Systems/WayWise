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
    if (!gimbal.isNull())
        ui->gimbalStatusLabel->setText("Gimbal found.");
}

QSharedPointer<MapModule> CameraGimbalUI::getSetRoiByClickOnMapModule() const
{
    return mSetRoiByClickOnMapModule;
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
