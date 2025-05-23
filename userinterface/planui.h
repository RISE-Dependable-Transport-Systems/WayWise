/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * UI building block for planning routes
 */

#ifndef PLANUI_H
#define PLANUI_H

#include <QWidget>
#include <QSharedPointer>
#include <QFileDialog>
#include <QMessageBox>
#include <QXmlStreamWriter>
#include <QInputDialog>
#include "userinterface/map/routeplannermodule.h"
#include "userinterface/routegeneratorui.h"
#include "communication/vehicleconnections/vehicleconnection.h"

namespace Ui {
class PlanUI;
}

class PlanUI : public QWidget
{
    Q_OBJECT

public:
    explicit PlanUI(QWidget *parent = nullptr);
    ~PlanUI();

    void setCurrentVehicleConnection(const QSharedPointer<VehicleConnection> &currentVehicleConnection);

    QSharedPointer<RoutePlannerModule> getRoutePlannerModule() const;
    QSharedPointer<RouteGeneratorUI> getRouteGeneratorUI() const;

signals:
    void routeDoneForUse(const QList<PosPoint>& route);

private slots:
    void on_addRouteButton_clicked();

    void on_removeRouteButton_clicked();

    void on_currentRouteSpinBox_valueChanged(int value);

    void on_sendToAutopilotButton_clicked();

    void on_heightSpinBox_valueChanged(double arg1);

    void on_speedSpinBox_valueChanged(double arg1);

    void on_attributeLineEdit_textChanged(const QString &arg1);

    void on_updatePointCheckBox_toggled(bool checked);

    void on_exportCurrentRouteButton_clicked();

    void on_exportAllRoutesButton_clicked();

    void on_importRouteButton_clicked();

    void on_generateRouteButton_clicked();

    void on_reverseButton_clicked();

    void on_appendButton_clicked();

    void on_splitButton_clicked();

    void on_downloadCurrentRouteFromVehicleButton_clicked();

private:
    Ui::PlanUI *ui;
    QSharedPointer<RoutePlannerModule> mRoutePlanner;
    QSharedPointer<RouteGeneratorUI> mRouteGeneratorUI;
    void xmlStreamWriteRoute(QXmlStreamWriter &xmlWriteStream, const QList<PosPoint> route);
    void xmlStreamWriteEnuRef(QXmlStreamWriter &xmlWriteStream, const llh_t enuRef);
    QSharedPointer<VehicleConnection> mCurrentVehicleConnection;
};

#endif // PLANUI_H
