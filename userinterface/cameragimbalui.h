#ifndef CAMERAGIMBALUI_H
#define CAMERAGIMBALUI_H

#include <QWidget>
#include <QSharedPointer>
#include "sensors/camera/gimbal.h"
#include "userinterface/map/mapwidget.h"

// TODO: quick test, refactor!
#include "communication/vehicleconnections/mavsdkstation.h"

namespace Ui {
class CameraGimbalUI;
}

class CameraGimbalUI : public QWidget
{
    Q_OBJECT

public:
    explicit CameraGimbalUI(QWidget *parent = nullptr);
    ~CameraGimbalUI();
    void setGimbal(const QSharedPointer<Gimbal> gimbal);
    QSharedPointer<MapModule> getSetRoiByClickOnMapModule() const;

    void setMavVehicleConnection(const QSharedPointer<MavsdkVehicleConnection> &mavVehicleConnection);

private slots:
    void on_actuatorTwoHighButton_clicked();
    void on_actuatorTwoMidButton_clicked();
    void on_actuatorTwoLowButton_clicked();
    void on_actuatorOneLowButton_clicked();
    void on_actuatorOneMidButton_clicked();
    void on_actuatorOneHighButton_clicked();
    void on_zeroButton_clicked();
    void on_upButton_clicked();
    void on_rightButton_clicked();
    void on_downButton_clicked();
    void on_leftButton_clicked();
    void on_doubleUpButton_clicked();
    void on_doubleRightButton_clicked();
    void on_doubleDownButton_clicked();
    void on_doubleLeftButton_clicked();
    void on_tripleUpButton_clicked();
    void on_tripleRightButton_clicked();
    void on_tripleDownButton_clicked();
    void on_trippleLeftButton_clicked();

    void on_yawFollowButton_clicked();

    void on_yawLockButton_clicked();

private:
    class SetRoiByClickOnMapModule : public MapModule {
        // MapModule interface
    public:
        explicit SetRoiByClickOnMapModule(CameraGimbalUI *parent);
        virtual void processPaint(QPainter &painter, int width, int height, bool highQuality, QTransform drawTrans, QTransform txtTrans, double scale) override;
        virtual QSharedPointer<QMenu> populateContextMenu(const xyz_t &mapPos, const llh_t &enuReference) override;
    private:
        CameraGimbalUI *mCameraGimbalUI;
        QSharedPointer<QMenu> mRoiContextMenu;
        QSharedPointer<QAction> mSetRoiAction;
        xyz_t mLastClickedMapPos;
        llh_t mLastEnuRefFromMap;
        xyz_t mLastRoiSet = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    };

    void moveGimbal(double pitch_deg, double yaw_deg);
    Ui::CameraGimbalUI *ui;
    QSharedPointer<Gimbal> mGimbal;
    QSharedPointer<SetRoiByClickOnMapModule> mSetRoiByClickOnMapModule;
    QSharedPointer<MavsdkVehicleConnection> mMavVehicleConnection;
    QPair<double, double> mPitchYawState = {0.0, 0.0};
    const double SMALL_STEP = 1.0;
    const double MEDIUM_STEP = 5.0;
    const double BIG_STEP = 20.0;
    // Values for Gremsy Pixy U (https://gremsy.com/pixy-u-manual)
    const QPair<double, double> YAW_RANGE = {-320.0, +320.0};
    const QPair<double, double> PITCH_RANGE = {-45.0, 135.0};
};

#endif // CAMERAGIMBALUI_H
