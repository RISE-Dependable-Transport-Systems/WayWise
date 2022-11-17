#ifndef ROUTEGENERATORUI_H
#define ROUTEGENERATORUI_H

#include <QDialog>
#include <QProxyStyle>
#include <QStyleOptionTab>
#include <QSharedPointer>
#include "userinterface/map/routeplannermodule.h"

namespace Ui {
class RouteGeneratorUI;
}

class RouteGeneratorUI : public QDialog
{
    Q_OBJECT

public:
    explicit RouteGeneratorUI(QWidget *parent = nullptr);
    ~RouteGeneratorUI();

    void setEnuRef(const llh_t &llh);

signals:
    void routeDoneForUse(const QList<PosPoint>& route);

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

    void on_cancelButton_clicked();

    void on_useRouteButton_clicked();

private:
    Ui::RouteGeneratorUI *ui;

    QSharedPointer<RoutePlannerModule> mRoutePlannerModule;
    const int boundRouteIndex = 0;
    const int generatedRouteIndex = 1;

    void updatePreviewRoute();

    // https://forum.qt.io/post/433000
    class CustomTabStyle : public QProxyStyle {
    public:
      QSize sizeFromContents(ContentsType type, const QStyleOption* option,
                             const QSize& size, const QWidget* widget) const {
        QSize s = QProxyStyle::sizeFromContents(type, option, size, widget);
        if (type == QStyle::CT_TabBarTab) {
          s.transpose();
        }
        return s;
      }

      void drawControl(ControlElement element, const QStyleOption* option, QPainter* painter, const QWidget* widget) const {
        if (element == CE_TabBarTabLabel) {
          if (const QStyleOptionTab* tab = qstyleoption_cast<const QStyleOptionTab*>(option)) {
            QStyleOptionTab opt(*tab);
            opt.shape = QTabBar::RoundedNorth;
            QProxyStyle::drawControl(element, &opt, painter, widget);
            return;
          }
        }
        QProxyStyle::drawControl(element, option, painter, widget);
      }
    };
};

#endif // ROUTEGENERATORUI_H
