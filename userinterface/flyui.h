/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * UI building block for flying a multicopter-type vehicle
 */

#ifndef FLYUI_H
#define FLYUI_H

#include <QWidget>
#include <QSharedPointer>
#include "communication/vehicleconnections/mavsdkvehicleconnection.h"
#include "autopilot/waypointfollower.h"
#include "userinterface/map/mapwidget.h"

namespace Ui {
class FlyUI;
}

class FlyUI : public QWidget
{
    Q_OBJECT

public:
    explicit FlyUI(QWidget *parent = nullptr);
    ~FlyUI();

    void setCurrentVehicleConnection(const QSharedPointer<MavsdkVehicleConnection> &currentVehicleConnection);
    QSharedPointer<MavsdkVehicleConnection> getCurrentVehicleConnection() const;
    QSharedPointer<MapModule> getGotoClickOnMapModule();

public slots:
    void gotRouteForAutopilot(const QList<PosPoint>& route);

    void updatePrecisionLandPosition(double, double, double);
    void updatePrecisionLandRequest(bool req);


private slots:
    void on_takeoffButton_clicked();

    void on_armButton_clicked();

    void on_disarmButton_clicked();

    void on_returnToHomeButton_clicked();

    void on_landButton_clicked();

    void on_gotoButton_clicked();

    void on_getPositionButton_clicked();

    void on_apRestartButton_clicked();

    void on_apStartButton_clicked();

    void on_apPauseButton_clicked();

    void on_apStopButton_clicked();


    void on_precisionLandButton_clicked();

private:
    class GotoClickOnMapModule : public MapModule {
        // MapModule interface
    public:
        explicit GotoClickOnMapModule(FlyUI *parent);
        virtual void processPaint(QPainter &painter, int width, int height, bool highQuality, QTransform drawTrans, QTransform txtTrans, double scale) override;
        virtual QSharedPointer<QMenu> populateContextMenu(const xyz_t &mapPos, const llh_t &enuReference) override;
    private:
        FlyUI *mFlyUI;
        QSharedPointer<QMenu> mGotoContextMenu;
        QSharedPointer<QAction> mGotoAction;
        xyz_t mLastClickedMapPos;
    };

    Ui::FlyUI *ui;
    QSharedPointer<GotoClickOnMapModule> mGotoClickOnMapModule;
    QSharedPointer<MavsdkVehicleConnection> mCurrentVehicleConnection;
    xyz_t mPrecisionLandPosition;
    bool prescisionLandRequest;
};

#endif // FLYUI_H
