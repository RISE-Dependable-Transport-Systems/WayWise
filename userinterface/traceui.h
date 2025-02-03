/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * UI building block for tracing the vehicles' different position types
 */

#ifndef TRACEUI_H
#define TRACEUI_H

#include <QWidget>
#include <QSharedPointer>
#include "vehicles/vehiclestate.h"
#include "userinterface/map/tracemodule.h"

namespace Ui {
class TraceUI;
}

class TraceUI : public QWidget
{
    Q_OBJECT

public:
    explicit TraceUI(QWidget *parent = nullptr);
    ~TraceUI();

    void setCurrentTraceVehicle(QSharedPointer<VehicleState> vehicle);

    QSharedPointer<TraceModule> getTraceModule() const;

private slots:
    void on_startTraceButton_clicked();

    void on_stopTraceButton_clicked();

    void on_clearTraceButton_clicked();

private:
    Ui::TraceUI *ui;
    QSharedPointer<TraceModule> mTracemodule;
};

#endif // TRACEUI_H
