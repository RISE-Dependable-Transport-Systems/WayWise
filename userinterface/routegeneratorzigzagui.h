/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#ifndef ROUTEGENERATORZIGZAGUI_H
#define ROUTEGENERATORZIGZAGUI_H

#include <QWidget>
#include <QSharedPointer>
#include "map/routeplannermodule.h"
#include "routeplanning/zigzagroutegenerator.h"
#include "userinterface/routegeneratorui.h"

namespace Ui {
class RouteGeneratorZigZagUI;
}

class RouteGeneratorZigZagUI : public QWidget
{
    Q_OBJECT

public:
    explicit RouteGeneratorZigZagUI(QSharedPointer<RoutePlannerModule> routePlannerModule, QWidget *parent = nullptr);
    ~RouteGeneratorZigZagUI();

private slots:
    void on_zigZagToolbox_currentChanged(int index);

    void on_resetBoundButton_clicked();

    void on_boundDoneButton_clicked();

    void on_speedStraightsSpinBox_valueChanged(double arg1);

    void on_speedTurnsSpinBox_valueChanged(double arg1);

    void on_rowSpacingSpinBox_valueChanged(double arg1);

    void on_visitEverySpinBox_valueChanged(int arg1);

    void on_attributeStraightEdit_textChanged(const QString &arg1);

    void on_attributeTurnEdit_textEdited(const QString &arg1);

    void on_attributeDistanceAfterTurnSpinBox_valueChanged(double arg1);

    void on_attributeDistanceBeforeTurnSpinBox_valueChanged(double arg1);

    void on_stepsForTurningSpinBox_valueChanged(int arg1);

    void on_forceTurnsIntoBoundsCheckBox_stateChanged(int arg1);

    void on_generateFrameCheckBox_stateChanged(int arg1);

private:
    Ui::RouteGeneratorZigZagUI *ui;
    QSharedPointer<RoutePlannerModule> mRoutePlannerModule;

    void updatePreviewRoute();
};

#endif // ROUTEGENERATORZIGZAGUI_H
