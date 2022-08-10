/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * UI building block for planning routes
 */

#ifndef PLANUI_H
#define PLANUI_H

#include <QWidget>
#include <QSharedPointer>
#include "userinterface/map/routeplannermodule.h"

namespace Ui {
class PlanUI;
}

class PlanUI : public QWidget
{
    Q_OBJECT

public:
    explicit PlanUI(QWidget *parent = nullptr);
    ~PlanUI();

    QSharedPointer<RoutePlannerModule> getRoutePlannerModule() const;

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

private:
    Ui::PlanUI *ui;
    QSharedPointer<RoutePlannerModule> mRoutePlanner;
};

#endif // PLANUI_H
