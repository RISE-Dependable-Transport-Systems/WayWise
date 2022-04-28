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

//    mousePosMap.setSpeed(mRoutePointSpeed);
//    mousePosMap.setTime(mRoutePointTime);
//    mousePosMap.setAttributes(mRoutePointAttributes);
//    mousePosMap.setId(mAnchorId);
//    mousePosMap.setHeight(mAnchorHeight);

    double routeDist = 0.0;
    int pointOnCurrRouteInd = getClosestPoint(mapPos, mRoutes[mPlannerState.currentRouteIndex], routeDist);
    bool routeFound = (routeDist * scale * 1000.0) < 20 && routeDist >= 0.0;

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
        if (keyboardModifiers == Qt::ControlModifier) {
            if (mouseButtons & Qt::LeftButton) {

            } else if (mouseButtons & Qt::RightButton) {
                if (routeFound) {
    //                mRoutes[mRouteNow][routeInd].setSpeed(mRoutePointSpeed);
    //                mRoutes[mRouteNow][routeInd].setTime(mRoutePointTime);
    //                mRoutes[mRouteNow][routeInd].setAttributes(mRoutePointAttributes);
                }
            }
        } else if (keyboardModifiers == Qt::ShiftModifier) {
            if (mouseButtons & Qt::LeftButton) {
                if (routeFound) {
                    mPlannerState.currentPointIndex = pointOnCurrRouteInd;
                    mRoutes[mPlannerState.currentRouteIndex][pointOnCurrRouteInd].setXY(mapPos.getX(), mapPos.getY());
                } else {
                    if (mRoutes[mPlannerState.currentRouteIndex].size() < 2 ||
                            mRoutes[mPlannerState.currentRouteIndex].last().getDistanceTo(mapPos) <
                            mRoutes[mPlannerState.currentRouteIndex].first().getDistanceTo(mapPos)) {
                        mRoutes[mPlannerState.currentRouteIndex].append(mapPos);
    //                    emit routePointAdded(mapPos);
                    } else {
                        mRoutes[mPlannerState.currentRouteIndex].prepend(mapPos);
                    }
                }
                emit requestRepaint();
                return true;
            } else if (mouseButtons & Qt::RightButton) {
                if (routeFound) {
                    mRoutes[mPlannerState.currentRouteIndex].removeAt(pointOnCurrRouteInd);
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
}

bool RoutePlannerModule::removeCurrentRoute()
{
    if (mRoutes.size() == 1)
        return false;

    removeRoute(mPlannerState.currentRouteIndex);

    return true;
}

void RoutePlannerModule::removeRoute(int index)
{
    mRoutes.removeAt(index);

    if (mPlannerState.currentRouteIndex == mRoutes.size())
        mPlannerState.currentRouteIndex--;
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

void RoutePlannerModule::addRoutePoint(double px, double py, double speed, QTime time)
{
    PosPoint pos;
    pos.setXY(px, py);
    pos.setSpeed(speed);
    pos.setTime(time);
    mRoutes[mPlannerState.currentRouteIndex].append(pos);
    emit requestRepaint();
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
            QTime t = route[i].getTime();
            pointLabel.sprintf("P: %d %s\n"
                               "%.1f km/h\n"
                               "%02d:%02d:%02d:%03d\n"
                               "A: %08X",
                               i, ((i == 0) ? "- start" : ((i == route.size()-1) ? "- end" : "")),
                               route[i].getSpeed() * 3.6,
                               t.hour(), t.minute(), t.second(), t.msec(),
                               route[i].getAttributes());

            pointLabelPos.setX(p.x() + 10 / scaleFactor);
            pointLabelPos.setY(p.y());
            painter.setTransform(txtTrans);
            pointLabelPos = drawTrans.map(pointLabelPos);
            pen.setColor(Qt::black);
            painter.setPen(pen);
            pointLabelRectangle.setCoords(pointLabelPos.x(), pointLabelPos.y() - 20,
                                          pointLabelPos.x() + 200, pointLabelPos.y() + 60);
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
