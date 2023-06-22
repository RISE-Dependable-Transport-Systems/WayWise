/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
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
    static const int boundRouteIndex = 0;
    static const int generatedRouteIndex = 1;

    explicit RouteGeneratorUI(QWidget *parent = nullptr);
    ~RouteGeneratorUI();

    void setEnuRef(const llh_t &llh);
    llh_t getEnuRef();

signals:
    void routeDoneForUse(const QList<PosPoint>& route);

private slots:
    void on_cancelButton_clicked();

    void on_useRouteButton_clicked();

private:
    Ui::RouteGeneratorUI *ui;

    QSharedPointer<RoutePlannerModule> mRoutePlannerModule;

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
