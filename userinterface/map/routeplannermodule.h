/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * MapModule that allows creating and interacting with routes on the map
 */

#ifndef ROUTEPLANNERMODULE_H
#define ROUTEPLANNERMODULE_H

#include "userinterface/map/mapwidget.h"

class RoutePlannerModule : public MapModule
{
public:
    RoutePlannerModule();

    // MapModule interface
    virtual void processPaint(QPainter &painter, int width, int height, bool highQuality, QTransform drawTrans, QTransform txtTrans, double scale) override;
    virtual bool processMouse(bool isPress, bool isRelease, bool isMove, bool isWheel, QPoint widgetPos, PosPoint mapPos, double wheelAngleDelta, Qt::KeyboardModifiers keyboardModifiers, Qt::MouseButtons mouseButtons, double scale) override;
    void setCurrentRouteIndex(int index);
    int getCurrentRouteIndex();
    void setDrawRouteText(bool draw);
    QList<PosPoint> getCurrentRoute();
    QList<PosPoint> getRoute(int index);
    int getNumberOfRoutes();
    void addNewRoute();
    void addRoute(QList<PosPoint> route);
    void appendRouteToCurrentRoute(QList<PosPoint> route);
    bool removeCurrentRoute();
    void clearCurrentRoute();
    void removeRoute(int index);
    void setNewPointHeight(double height);
    void setNewPointSpeed(double speed);
    void setNewPointTime(QTime time);
    void setNewPointAttribute(uint32_t attribute);
    void setUpdatePointOnClick(bool update);
    void reverseRoute();

private:
    typedef enum {
        Default,
        Inactive,
        LAST
    } RoutePointType;

    struct {
        int currentRouteIndex = 0;
        int currentPointIndex = -1;
        bool drawRouteText = true;
        bool updatePointOnClick = true;
        double newPointHeight = 0.0;
        double newPointSpeed = 0.5; // [m/s]
        QTime newPointTime;
        uint32_t newPointAttribute = 0;

    } mPlannerState;

    void drawRoute(QPainter &painter, QTransform drawTrans, QTransform txtTrans, bool highQuality, double scaleFactor, const QList<PosPoint> &route, int routeID, bool isSelected, bool drawAnnotations);
    void drawCircleFast(QPainter &painter, QPointF center, double radius, int type);

    QList<QPixmap> mPixmaps;
    QList<QList<PosPoint> > mRoutes;
    int getClosestPoint(PosPoint p, QList<PosPoint> points, double &dist);
};

#endif // ROUTEPLANNERMODULE_H
