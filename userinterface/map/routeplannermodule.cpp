/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "routeplannermodule.h"

RoutePlannerModule::RoutePlannerModule()
{
    // Pre-render some things for speed
    for (int i = 0; i < RoutePointType::LAST; i++) {
        QPixmap pix(24, 24);
        pix.fill(Qt::transparent);
        QPainter p(&pix);

        QPen pen;
        pen.setWidth(4);

        switch (i) {
        case RoutePointType::Default: {
            pen.setColor(Qt::darkYellow);
            p.setBrush(Qt::yellow);
            p.setPen(pen);
            p.drawEllipse(2, 2, 20, 20);
        } break;

        case RoutePointType::Inactive: {
            pen.setColor(Qt::darkGray);
            p.setBrush(Qt::gray);
            p.setPen(pen);
            p.drawEllipse(2, 2, 20, 20);
        } break;
        }

        mPixmaps.append(pix);
    }

    mRoutes.clear();
    mRoutes.append(QList<PosPoint>());
}

void RoutePlannerModule::processPaint(QPainter &painter, int width, int height, bool highQuality, QTransform drawTrans, QTransform txtTrans, double scale)
{
    Q_UNUSED(width)
    Q_UNUSED(height)
    for (int rn = 0; rn < mRoutes.size(); rn++) {
        drawRoute(painter, drawTrans, txtTrans, highQuality, scale, mRoutes[rn], rn, rn == mPlannerState.currentRouteIndex, mPlannerState.drawRouteText);
    }
}

bool RoutePlannerModule::processMouse(bool isPress, bool isRelease, bool isMove, bool isWheel, QPoint widgetPos, PosPoint mapPos,
                                      double wheelAngleDelta, Qt::KeyboardModifiers keyboardModifiers, Qt::MouseButtons mouseButtons, double scale)
{
    Q_UNUSED(isWheel)
    Q_UNUSED(widgetPos)
    Q_UNUSED(wheelAngleDelta)

    double routeDist = 0.0;
    int closestPointOnCurrRouteInd = getClosestPoint(mapPos, mRoutes[mPlannerState.currentRouteIndex], routeDist);
    bool clickedOnPoint = (routeDist * scale * 1000.0) < 20 && routeDist >= 0.0;

    if (isRelease) {
        mPlannerState.currentPointIndex = -1;
        return false;
    }

    if (isMove) {
        if (mPlannerState.currentPointIndex >= 0) {
            mRoutes[mPlannerState.currentRouteIndex][mPlannerState.currentPointIndex].setXY(mapPos.getX(), mapPos.getY());
            emit requestRepaint();
            return true;
        }
        return false;
    }

    if (isPress) {
        if (keyboardModifiers == Qt::ShiftModifier) {
            if (mouseButtons & Qt::LeftButton) {
                if (clickedOnPoint) { // update existing point
                    if (mPlannerState.updatePointOnClick) {
                        mPlannerState.currentPointIndex = closestPointOnCurrRouteInd;
                        mRoutes[mPlannerState.currentRouteIndex][closestPointOnCurrRouteInd].setXYZ({mapPos.getX(), mapPos.getY(), mPlannerState.newPointHeight});
                        mRoutes[mPlannerState.currentRouteIndex][closestPointOnCurrRouteInd].setSpeed(mPlannerState.newPointSpeed);
                        mRoutes[mPlannerState.currentRouteIndex][closestPointOnCurrRouteInd].setTime(mPlannerState.newPointTime);
                        mRoutes[mPlannerState.currentRouteIndex][closestPointOnCurrRouteInd].setAttributes(mPlannerState.newPointAttribute);
                    }
                } else { // create new point
                    PosPoint newPoint;
                    newPoint.setXYZ({mapPos.getX(), mapPos.getY(), mPlannerState.newPointHeight});
                    newPoint.setSpeed(mPlannerState.newPointSpeed);
                    newPoint.setTime(mPlannerState.newPointTime);
                    newPoint.setAttributes(mPlannerState.newPointAttribute);

                    // some hard to read logic to determine where in the route to insert (before or after closest point?) incl. special cases
                    if (mRoutes[mPlannerState.currentRouteIndex].size() < 2)
                        mRoutes[mPlannerState.currentRouteIndex].append(newPoint);
                    else if (closestPointOnCurrRouteInd == 0) {
                        if (mRoutes[mPlannerState.currentRouteIndex].at(closestPointOnCurrRouteInd + 1).getDistanceTo(mapPos) <
                                mRoutes[mPlannerState.currentRouteIndex].at(closestPointOnCurrRouteInd).getDistanceTo(mRoutes[mPlannerState.currentRouteIndex].at(closestPointOnCurrRouteInd + 1)))
                            mRoutes[mPlannerState.currentRouteIndex].insert(closestPointOnCurrRouteInd + 1, newPoint);
                        else
                            mRoutes[mPlannerState.currentRouteIndex].insert(closestPointOnCurrRouteInd, newPoint);
                    } else if (closestPointOnCurrRouteInd == mRoutes[mPlannerState.currentRouteIndex].size() - 1) {
                        if (mRoutes[mPlannerState.currentRouteIndex].at(closestPointOnCurrRouteInd - 1).getDistanceTo(mapPos) <
                                mRoutes[mPlannerState.currentRouteIndex].at(closestPointOnCurrRouteInd).getDistanceTo(mRoutes[mPlannerState.currentRouteIndex].at(closestPointOnCurrRouteInd - 1)))
                            mRoutes[mPlannerState.currentRouteIndex].insert(closestPointOnCurrRouteInd, newPoint);
                        else
                            mRoutes[mPlannerState.currentRouteIndex].insert(closestPointOnCurrRouteInd + 1, newPoint);
                    } else { // "standard case" somewhere on the route
                        if (mRoutes[mPlannerState.currentRouteIndex].at(closestPointOnCurrRouteInd - 1).getDistanceTo(mapPos) <
                                mRoutes[mPlannerState.currentRouteIndex].at(closestPointOnCurrRouteInd + 1).getDistanceTo(mapPos))
                            mRoutes[mPlannerState.currentRouteIndex].insert(closestPointOnCurrRouteInd, newPoint);
                        else
                            mRoutes[mPlannerState.currentRouteIndex].insert(closestPointOnCurrRouteInd + 1, newPoint);
                    }
                }
                emit requestRepaint();
                return true;
            } else if (mouseButtons & Qt::RightButton) {
                if (clickedOnPoint) {
                    mRoutes[mPlannerState.currentRouteIndex].removeAt(closestPointOnCurrRouteInd);
                } else {
    //                removeLastRoutePoint();
                }
                emit requestRepaint();
                return true;
            }
        }
        return false;
    }

    return false;
}

void RoutePlannerModule::setCurrentRouteIndex(int index)
{
    if (index >= 0 && index < mRoutes.size())
        mPlannerState.currentRouteIndex = index;
    emit requestRepaint();
}

int RoutePlannerModule::getCurrentRouteIndex()
{
    return mPlannerState.currentRouteIndex;
}

void RoutePlannerModule::setDrawRouteText(bool draw)
{
    mPlannerState.drawRouteText = draw;
}

QList<PosPoint> RoutePlannerModule::getCurrentRoute()
{
    return getRoute(mPlannerState.currentRouteIndex);
}

QList<PosPoint> RoutePlannerModule::getRoute(int index)
{
    return mRoutes.at(index);
}

int RoutePlannerModule::getNumberOfRoutes()
{
    return mRoutes.size();
}

void RoutePlannerModule::addNewRoute()
{
    addRoute(QList<PosPoint>());
}

void RoutePlannerModule::addRoute(QList<PosPoint> route)
{
    mRoutes.append(route);
    emit requestRepaint();
}

void RoutePlannerModule::appendRouteToCurrentRoute(QList<PosPoint> route)
{
    mRoutes[mPlannerState.currentRouteIndex].append(route);
    emit requestRepaint();
}

bool RoutePlannerModule::removeCurrentRoute()
{
    if (mRoutes.size() == 1)
        return false;

    removeRoute(mPlannerState.currentRouteIndex);

    return true;
}

void RoutePlannerModule::clearCurrentRoute()
{
    mRoutes[mPlannerState.currentRouteIndex].clear();
    emit requestRepaint();
}

void RoutePlannerModule::removeRoute(int index)
{
    mRoutes.removeAt(index);

    if (mPlannerState.currentRouteIndex == mRoutes.size())
        mPlannerState.currentRouteIndex--;
}

void RoutePlannerModule::setNewPointHeight(double height)
{
    mPlannerState.newPointHeight = height;
}

void RoutePlannerModule::setNewPointSpeed(double speed)
{
    mPlannerState.newPointSpeed = speed;
}

void RoutePlannerModule::setNewPointTime(QTime time)
{
    mPlannerState.newPointTime = time;
}

void RoutePlannerModule::setNewPointAttribute(uint32_t attribute)
{
    mPlannerState.newPointAttribute = attribute;
}

void RoutePlannerModule::setUpdatePointOnClick(bool update)
{
    mPlannerState.updatePointOnClick = update;
}

int RoutePlannerModule::getClosestPoint(PosPoint p, QList<PosPoint> points, double &dist)
{
    int closest = -1;
    dist = -1.0;
    for (int i = 0;i < points.size();i++) {
        double d = points[i].getDistanceTo(p);
        if (dist < 0.0 || d < dist) {
            dist = d;
            closest = i;
        }
    }

    return closest;
}

void RoutePlannerModule::drawCircleFast(QPainter &painter, QPointF center, double radius, int type)
{
    painter.drawPixmap(center.x() - radius, center.y() - radius,
                       2.0 * radius, 2.0 * radius, mPixmaps.at(type));
}

void RoutePlannerModule::drawRoute(QPainter& painter, QTransform drawTrans, QTransform txtTrans, bool highQuality, double scaleFactor, const QList<PosPoint> &route, int routeID, bool isSelected, bool drawAnnotations)
{

    Qt::GlobalColor defaultDarkColor = Qt::darkGray;
    Qt::GlobalColor defaultColor = Qt::gray;
    if (isSelected) {
        defaultDarkColor = Qt::darkYellow;
        defaultColor = Qt::yellow;
    }

    QPen pen = painter.pen();
    pen.setWidthF(5.0 / scaleFactor);
    painter.setTransform(drawTrans);

    for (int i = 1;i < route.size();i++) {
        pen.setColor(defaultDarkColor);
        painter.setBrush(defaultColor);
        painter.setPen(pen);

        painter.setOpacity(0.7);
        painter.drawLine(route.at(i-1).getX() * 1000.0, route.at(i-1).getY() * 1000.0,
                         route.at(i).getX() * 1000.0, route.at(i).getY() * 1000.0);
        painter.setOpacity(1.0);
    }

    for (int i = 0;i < route.size();i++) {
        QString pointLabel;
        QPointF pointLabelPos;
        QRectF pointLabelRectangle;
        QPointF p = route[i].getPointMm();

        painter.setTransform(drawTrans);

        if (highQuality) {
            if (isSelected) {
                pen.setColor(Qt::darkYellow);
                painter.setBrush(Qt::yellow);
            } else {
                pen.setColor(Qt::darkGray);
                painter.setBrush(Qt::gray);
            }

            pen.setWidthF(3.0 / scaleFactor);
            painter.setPen(pen);

            painter.drawEllipse(p, 10.0 / scaleFactor,
                                10.0 / scaleFactor);
        } else {
            drawCircleFast(painter, p, 10.0 / scaleFactor, isSelected ? 0 : 1); // TODO generalize / refactor (by color?)
        }

        // Draw highlight for first and last point in active route
        if (isSelected && (i == 0 || i == route.size()-1)) {
            QPointF ptmp;
            ptmp.setX(p.x() - 7.0 / scaleFactor);
            ptmp.setY(p.y() + 7.0 / scaleFactor);
            if (i == 0) {
                pen.setColor(Qt::green);
                painter.setBrush(Qt::darkGreen);
            } else {
                pen.setColor(Qt::red);
                painter.setBrush(Qt::darkRed);
            }
            pen.setWidthF(2.0 / scaleFactor);
            painter.setPen(pen);
            painter.drawEllipse(ptmp, 5.0 / scaleFactor,
                                5.0 / scaleFactor);
        }

        // Draw text only for selected route
        if (isSelected && drawAnnotations) {
            //QTime t = route[i].getTime();
            pointLabel.sprintf("P: %d %s\n"
                               "(%.2f, %.2f, %.2f)\n"
                               "%.1f km/h\n"
                               //"%02d:%02d:%02d:%03d\n"
                               "A: %08X",
                               i, ((i == 0) ? "- start" : ((i == route.size()-1) ? "- end" : "")),
                               route[i].getX(), route[i].getY(), route[i].getHeight(),
                               route[i].getSpeed() * 3.6,
                               //t.hour(), t.minute(), t.second(), t.msec(),
                               route[i].getAttributes());

            pointLabelPos.setX(p.x() + 10 / scaleFactor);
            pointLabelPos.setY(p.y());
            painter.setTransform(txtTrans);
            pointLabelPos = drawTrans.map(pointLabelPos);
            pen.setColor(Qt::black);
            painter.setPen(pen);
            pointLabelRectangle.setCoords(pointLabelPos.x(), pointLabelPos.y() - 20,
                                          pointLabelPos.x() + 500, pointLabelPos.y() + 100);
            painter.drawText(pointLabelRectangle, pointLabel);
        } else {
            pointLabel.sprintf("%d", routeID);
            pointLabelPos.setX(p.x());
            pointLabelPos.setY(p.y());
            painter.setTransform(txtTrans);
            pointLabelPos = drawTrans.map(pointLabelPos);
            pen.setColor(Qt::black);
            painter.setPen(pen);
            pointLabelRectangle.setCoords(pointLabelPos.x() - 20, pointLabelPos.y() - 20,
                                          pointLabelPos.x() + 20, pointLabelPos.y() + 20);
            painter.drawText(pointLabelRectangle, Qt::AlignCenter, pointLabel);
        }
    }
}
