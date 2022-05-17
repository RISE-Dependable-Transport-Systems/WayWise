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
    void on_pushButton_11_clicked();

private slots:
    void on_pushButton_10_clicked();

private slots:
    void on_pushButton_9_clicked();

private slots:
    void on_pushButton_8_clicked();

private slots:
    void on_pushButton_clicked();

private slots:
    void on_actuatorTwoHighButton_clicked();

private slots:
    void on_actuatorTwoMidButton_clicked();

private slots:
    void on_actuatorTwoLowButton_clicked();

private slots:

    void on_actuatorOneLowButton_clicked();

    void on_actuatorOneMidButton_clicked();

    void on_actuatorOneHighButton_clicked();

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

    Ui::CameraGimbalUI *ui;
    QSharedPointer<Gimbal> mGimbal;
    QSharedPointer<SetRoiByClickOnMapModule> mSetRoiByClickOnMapModule;
    QSharedPointer<MavsdkVehicleConnection> mMavVehicleConnection;
    QPair<double, double> mPitchYawState = {0.0, 0.0};
};

#endif // CAMERAGIMBALUI_H
