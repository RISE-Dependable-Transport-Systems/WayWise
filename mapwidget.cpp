/*
    Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se
              2020        Marvin Damschen  marvin.damschen@ri.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * TODO
 * - gnss, uwb, etc pos refactoring
 * - gather map preferences in class/struct, same with status
 * - make utility functions public/move
 * - interface to register mouse, keyboard actions
 */

#include <QDebug>
#include <math.h>
#include <qmath.h>
#include <QPrinter>
#include <QPrintEngine>
#include <QTime>

#include "mapwidget.h"

// Some utility functions
namespace
{
static void normalizeAngleRad(double &angle)
{
    angle = fmod(angle, 2.0 * M_PI);

    if (angle < 0.0) {
        angle += 2.0 * M_PI;
    }
}

void minMaxEps(double x, double y, double &min, double &max) {
    double eps;

    if (fabs(x) > fabs(y)) {
        eps = fabs(x) / 1e10;
    } else {
        eps = fabs(y) / 1e10;
    }

    if (x > y) {
        max = x + eps;
        min = y - eps;
    } else {
        max = y + eps;
        min = x - eps;
    }
}

// See https://www.topcoder.com/community/data-science/data-science-tutorials/geometry-concepts-line-intersection-and-its-applications/
bool lineSegmentIntersection(const QPointF &p1, const QPointF &p2, const QPointF &q1, const QPointF &q2) {
    bool res = false;

    const double A1 = p2.y() - p1.y();
    const double B1 = p1.x() - p2.x();
    const double C1 = A1 * p1.x() + B1 * p1.y();

    const double A2 = q2.y() - q1.y();
    const double B2 = q1.x() - q2.x();
    const double C2 = A2 * q1.x() + B2 * q1.y();

    const double det = A1 * B2 - A2 * B1;

    if(fabs(det) < 1e-6) {
        //Lines are parallel
    } else {
        double x = (B2 * C1 - B1 * C2) / det;
        double y = (A1 * C2 - A2 * C1) / det;

        // Check if this point is on both line segments.
        double p1XMin, p1XMax, p1YMin, p1YMax;
        minMaxEps(p1.x(), p2.x(), p1XMin, p1XMax);
        minMaxEps(p1.y(), p2.y(), p1YMin, p1YMax);

        double q1XMin, q1XMax, q1YMin, q1YMax;
        minMaxEps(q1.x(), q2.x(), q1XMin, q1XMax);
        minMaxEps(q1.y(), q2.y(), q1YMin, q1YMax);

        if (    x <= p1XMax && x >= p1XMin &&
                y <= p1YMax && y >= p1YMin &&
                x <= q1XMax && x >= q1XMin &&
                y <= q1YMax && y >= q1YMin) {
            res = true;
        }
    }

    return res;
}

bool isLineSegmentWithinRect(const QPointF &p1, const QPointF &p2, const double& xStart, const double &xEnd, const double& yStart, const double& yEnd) {
    QPointF q1(xStart, yStart);
    QPointF q2(xEnd, yStart);

    bool res = lineSegmentIntersection(p1, p2, q1, q2);

    if (!res) {
        q1.setX(xStart);
        q1.setY(yStart);
        q2.setX(xStart);
        q2.setY(yEnd);
        res = lineSegmentIntersection(p1, p2, q1, q2);
    }

    if (!res) {
        q1.setX(xStart);
        q1.setY(yEnd);
        q2.setX(xEnd);
        q2.setY(yEnd);
        res = lineSegmentIntersection(p1, p2, q1, q2);
    }

    if (!res) {
        q1.setX(xEnd);
        q1.setY(yStart);
        q2.setX(xEnd);
        q2.setY(yEnd);
        res = lineSegmentIntersection(p1, p2, q1, q2);
    }

    return res;
}

bool isLineSegmentWithinRect(const QPointF &p1, const QPointF &p2, const QRectF& rect) {
    const double xStart = rect.left();
    const double xEnd = rect.right();
    const double yStart = rect.bottom();
    const double yEnd = rect.top();

    return isLineSegmentWithinRect(p1, p2, xStart, xEnd, yStart, yEnd);
}
}

MapWidget::MapWidget(QWidget *parent) : QWidget(parent)
{
    mScaleFactor = 0.1;
    mRotation = 0;
    mXOffset = 0;
    mYOffset = 0;
    mMouseLastX = 1000000;
    mMouseLastY = 1000000;
    mFollowObjectId = -1;
    mTraceObject = -1;
    mSelectedObject = -1;
    xRealPos = 0;
    yRealPos = 0;
    mRoutePointSpeed = 1.0;
	mRoutePointTime = QTime();
    mRoutePointAttributes = 0;
    mAnchorId = 0;
    mAnchorHeight = 1.0;
    mAntialiasDrawings = false;
    mAntialiasOsm = true;
    mInfoTraceTextZoom = 0.5;
    mDrawGrid = true;
    mRoutePointSelected = -1;
    mAnchorSelected = -1;
    mRouteNow = 0;
    mTraceMinSpaceObject = 0.05;
    mTraceMinSpaceGps = 0.05;
    mInfoTraceNow = 0;
    mAnchorMode = false;
    mDrawRouteText = true;
    mDrawUwbTrace = false;
    mCameraImageWidth = 0.46;
    mCameraImageOpacity = 0.8;

    mOsm = new OsmClient(this);
    mDrawOpenStreetmap = true;
    mOsmZoomLevel = 15;
    mOsmRes = 1.0;
    mOsmMaxZoomLevel = 19;
    mDrawOsmStats = false;

    mRoutes.clear();
    QList<PosPoint> l;
    mRoutes.append(l);

    mInfoTraces.clear();
    mInfoTraces.append(l);

    mTimer = new QTimer(this);
    mTimer->start(20);

    // ASTA
    mRefLlh = {57.78100308, 12.76925422, 253.76};

    // RISE RTK base station
//    mRefLat = 57.71495867;
//    mRefLon = 12.89134921;
//    mRefHeight = 219.0;

    // Hardcoded for now
    mOsm->setCacheDir("osm_tiles");
    //    mOsm->setTileServerUrl("http://tile.openstreetmap.org");
    mOsm->setTileServerUrl("http://c.osm.rrze.fau.de/osmhd"); // Also https

    connect(mOsm, SIGNAL(tileReady(OsmTile)), this, SLOT(tileReady(OsmTile)));
    connect(mOsm, SIGNAL(errorGetTile(QString)), this, SLOT(errorGetTile(QString)));
    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));

    setMouseTracking(true);

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

    grabGesture(Qt::PinchGesture);
}

QSharedPointer<VehicleState> MapWidget::getVehicleState(int vehicleID)
{
	for (int i = 0;i < mObjectStateList.size();i++) {
		if (mObjectStateList[i]->getId() == vehicleID) {
			return mObjectStateList[i].dynamicCast<VehicleState>();
        }
    }

    return 0;
}

void MapWidget::addObject(QSharedPointer<ObjectState> objectState)
{
	mObjectStateList.append(objectState);
	connect(objectState.get(), &ObjectState::positionUpdated, this, &MapWidget::objectPositionUpdated);
}
void MapWidget::addVehicle(QSharedPointer<VehicleState> vehicleState)
{
	mObjectStateList.append(vehicleState.dynamicCast<ObjectState>());
	connect(vehicleState.get(), &VehicleState::positionUpdated, this, &MapWidget::objectPositionUpdated);
}

bool MapWidget::removeObject(int objectID)
{
    const auto oldSize = mObjectStateList.size();
    std::remove_if(mObjectStateList.begin(), mObjectStateList.end(),
                   [objectID](const QSharedPointer<ObjectState>& st){
        return st->getId() == objectID;
    });
    return oldSize != mObjectStateList.size();
}

bool MapWidget::removeVehicle(int vehicleID)
{
    const auto oldSize = mObjectStateList.size();
    std::remove_if(mObjectStateList.begin(), mObjectStateList.end(),
                   [vehicleID](const QSharedPointer<ObjectState>& st){
        return st->getId() == vehicleID && st.dynamicCast<VehicleState>() != nullptr;
    });
    return oldSize != mObjectStateList.size();
}

void MapWidget::clearObjects()
{
	mObjectStateList.clear();
}

void MapWidget::clearVehicles()
{
    clearObjects(); // TODO only remove vehicle type objects
}

PosPoint *MapWidget::getAnchor(int anchorId)
{
    for (int i = 0;i < mAnchors.size();i++) {
        if (mAnchors[i].getId() == anchorId) {
            return &mAnchors[i];
        }
    }

    return 0;
}

void MapWidget::addAnchor(const PosPoint &anchor)
{
    mAnchors.append(anchor);
    update();
}

bool MapWidget::removeAnchor(int anchorId)
{
    bool found = false;

    QMutableListIterator<PosPoint> i(mAnchors);
    while (i.hasNext()) {
        if (i.next().getId() == anchorId) {
            i.remove();
            found = true;
        }
    }

    return found;
}

void MapWidget::clearAnchors()
{
    mAnchors.clear();
    update();
}

QList<PosPoint> MapWidget::getAnchors()
{
    return mAnchors;
}

void MapWidget::setScaleFactor(double scale)
{
    double scaleDiff = scale / mScaleFactor;
    mScaleFactor = scale;
    mXOffset *= scaleDiff;
    mYOffset *= scaleDiff;
    update();
}

double MapWidget::getScaleFactor()
{
    return mScaleFactor;
}

void MapWidget::setRotation(double rotation)
{
    mRotation = rotation;
    update();
}

void MapWidget::setXOffset(double offset)
{
    mXOffset = offset;
    update();
}

void MapWidget::setYOffset(double offset)
{
    mYOffset = offset;
    update();
}

void MapWidget::moveView(double px, double py)
{
    PosPoint followLoc;
    followLoc.setXY(px, py);
    mXOffset = -followLoc.getX() * 1000.0 * mScaleFactor;
    mYOffset = -followLoc.getY() * 1000.0 * mScaleFactor;
    update();
}

void MapWidget::clearTrace()
{
    mObjectTrace.clear();
    mObjectTraceGNSS.clear();
    mObjectTraceUwb.clear();
    update();
}

void MapWidget::addRoutePoint(double px, double py, double speed, QTime time)
{
    PosPoint pos;
    pos.setXY(px, py);
    pos.setSpeed(speed);
    pos.setTime(time);
    mRoutes[mRouteNow].append(pos);
    update();
}

QList<PosPoint> MapWidget::getRoute(int ind)
{
    if (ind < 0) {
        return mRoutes[mRouteNow];
    } else {
        if (mRoutes.size() > ind) {
            return mRoutes[ind];
        } else {
            QList<PosPoint> tmp;
            return tmp;
        }
    }
}

QList<QList<PosPoint> > MapWidget::getRoutes()
{
    return mRoutes;
}

void MapWidget::setRoute(const QList<PosPoint> &route)
{
    mRoutes[mRouteNow] = route;
    update();
}

void MapWidget::addRoute(const QList<PosPoint> &route)
{
    while (!mRoutes.isEmpty() &&
           mRoutes.last().isEmpty() &&
           mRouteNow < mRoutes.size()) {
        mRoutes.removeLast();
    }

    mRoutes.append(route);
    update();
}

int MapWidget::getRouteNum()
{
    return mRoutes.size();
}

void MapWidget::clearRoute()
{
    mRoutes[mRouteNow].clear();
    update();
}

void MapWidget::clearAllRoutes()
{
    for (int i = 0;i < mRoutes.size();i++) {
        mRoutes[i].clear();
    }

    update();
}

void MapWidget::setRoutePointSpeed(double speed)
{
    mRoutePointSpeed = speed;
}

void MapWidget::addInfoPoint(PosPoint &info)
{
    mInfoTraces[mInfoTraceNow].append(info);
    update();
}

void MapWidget::clearInfoTrace()
{
    mInfoTraces[mInfoTraceNow].clear();
    update();
}

void MapWidget::clearAllInfoTraces()
{
    for (int i = 0;i < mInfoTraces.size();i++) {
        mInfoTraces[i].clear();
    }

    update();
}

QPoint MapWidget::getMousePosRelative()
{
    QPoint p = this->mapFromGlobal(QCursor::pos());
    p.setX((p.x() - mXOffset - width() / 2) / mScaleFactor);
    p.setY((-p.y() - mYOffset + height() / 2) / mScaleFactor);
    return p;
}

void MapWidget::setAntialiasDrawings(bool antialias)
{
    mAntialiasDrawings = antialias;
    update();
}

void MapWidget::setAntialiasOsm(bool antialias)
{
    mAntialiasOsm = antialias;
    update();
}

void MapWidget::tileReady(OsmTile tile)
{
    (void)tile;
    update();
}

void MapWidget::errorGetTile(QString reason)
{
    qWarning() << "OSM tile error:" << reason;
}

void MapWidget::timerSlot()
{
    updateTraces();
}

void MapWidget::objectPositionUpdated()
{
    update();
}

void MapWidget::setFollowObject(int objectID)
{
    int oldObject = mFollowObjectId;
    mFollowObjectId = objectID;

    if (oldObject != mFollowObjectId) {
        update();
    }
}

void MapWidget::setTraceObject(int objectID)
{
    mTraceObject = objectID;
}

void MapWidget::setSelectedObject(int objectID)
{
    int oldObject = mSelectedObject;
    mSelectedObject = objectID;

    if (oldObject != mSelectedObject) {
        update();
    }
}

void MapWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    paint(painter, event->rect().width(), event->rect().height());
}

void MapWidget::mouseMoveEvent(QMouseEvent *e)
{
    bool ctrl = e->modifiers() == Qt::ControlModifier;
    bool shift = e->modifiers() == Qt::ShiftModifier;
    bool ctrl_shift = e->modifiers() == (Qt::ControlModifier | Qt::ShiftModifier);

    QPoint mousePosWidget = this->mapFromGlobal(QCursor::pos());
    PosPoint mousePosMap;
    QPoint p = getMousePosRelative();
    mousePosMap.setXY(p.x() / 1000.0, p.y() / 1000.0);

    for (const auto& m: mMapModules) {
        if (m->processMouse(false, false, true, false,
                            mousePosWidget, mousePosMap, 0.0,
                            e->modifiers(),
                            e->buttons(),
                            mScaleFactor)) {
            return;
        }
    }

    if (e->buttons() & Qt::LeftButton && !ctrl && !shift && !ctrl_shift) {
        int x = e->pos().x();
        int y = e->pos().y();

        if (mMouseLastX < 100000)
        {
            int diffx = x - mMouseLastX;
            mXOffset += diffx;
            update();
        }

        if (mMouseLastY < 100000)
        {
            int diffy = y - mMouseLastY;
            mYOffset -= diffy;

            emit offsetChanged(mXOffset, mYOffset);
            update();
        }

        mMouseLastX = x;
        mMouseLastY = y;
    }

    if (mRoutePointSelected >= 0) {
        mRoutes[mRouteNow][mRoutePointSelected].setXY(mousePosMap.getX(), mousePosMap.getY());
        update();
    }

    if (mAnchorSelected >= 0) {
        mAnchors[mAnchorSelected].setXY(mousePosMap.getX(), mousePosMap.getY());
        update();
    }

    updateClosestInfoPoint();
}

void MapWidget::mousePressEvent(QMouseEvent *e)
{
    setFocus();

    bool ctrl = e->modifiers() == Qt::ControlModifier;
    bool shift = e->modifiers() == Qt::ShiftModifier;
    bool ctrl_shift = e->modifiers() == (Qt::ControlModifier | Qt::ShiftModifier);

    QPoint mousePosWidget = this->mapFromGlobal(QCursor::pos());
    PosPoint mousePosMap;
    QPoint p = getMousePosRelative();
    mousePosMap.setXY(p.x() / 1000.0, p.y() / 1000.0);

    for (const auto& m: mMapModules) {
        if (m->processMouse(true, false, false, false,
                            mousePosWidget, mousePosMap, 0.0,
                            e->modifiers(),
                            e->buttons(),
                            mScaleFactor)) {
            return;
        }
    }

    mousePosMap.setSpeed(mRoutePointSpeed);
    mousePosMap.setTime(mRoutePointTime);
    mousePosMap.setAttributes(mRoutePointAttributes);
    mousePosMap.setId(mAnchorId);
    mousePosMap.setHeight(mAnchorHeight);

    double routeDist = 0.0;
    double anchorDist = 0.0;
    int routeInd = getClosestPoint(mousePosMap, mRoutes[mRouteNow], routeDist);
    int anchorInd = getClosestPoint(mousePosMap, mAnchors, anchorDist);
    bool routeFound = (routeDist * mScaleFactor * 1000.0) < 20 && routeDist >= 0.0;
    bool anchorFound = (anchorDist * mScaleFactor * 1000.0) < 20 && anchorDist >= 0.0;

    if (ctrl) {
        if (e->buttons() & Qt::LeftButton) {
            if (mSelectedObject >= 0) {
				for (int i = 0;i < mObjectStateList.size();i++) {
					QSharedPointer<VehicleState> vehicleState = mObjectStateList[i].dynamicCast<VehicleState>();
                    if (vehicleState->getId() == mSelectedObject) {
						PosPoint pos = vehicleState->getPosition();
                        QPoint p = getMousePosRelative();
                        pos.setXY(p.x() / 1000.0, p.y() / 1000.0);
						vehicleState->setPosition(pos);
                        emit posSet(mSelectedObject, pos);
                    }
                }
            }
        } else if (e->buttons() & Qt::RightButton) {
            if (mAnchorMode) {
                if (anchorFound) {
                    mAnchors[anchorInd].setId(mAnchorId);
                    mAnchors[anchorInd].setHeight(mAnchorHeight);
                }
            } else {
                if (routeFound) {
                    mRoutes[mRouteNow][routeInd].setSpeed(mRoutePointSpeed);
                    mRoutes[mRouteNow][routeInd].setTime(mRoutePointTime);
                    mRoutes[mRouteNow][routeInd].setAttributes(mRoutePointAttributes);
                }
            }
        }
        update();
    } else if (shift) {
        if (mAnchorMode) {
            if (e->buttons() & Qt::LeftButton) {
                if (anchorFound) {
                    mAnchorSelected = anchorInd;
                    mAnchors[anchorInd].setXY(mousePosMap.getX(), mousePosMap.getY());
                } else {
                    mAnchors.append(mousePosMap);
                }
            } else if (e->buttons() & Qt::RightButton) {
                if (anchorFound) {
                    mAnchors.removeAt(anchorInd);
                }
            }
        } else {
            if (e->buttons() & Qt::LeftButton) {
                if (routeFound) {
                    mRoutePointSelected = routeInd;
                    mRoutes[mRouteNow][routeInd].setXY(mousePosMap.getX(), mousePosMap.getY());
                } else {
                    if (mRoutes[mRouteNow].size() < 2 ||
                            mRoutes[mRouteNow].last().getDistanceTo(mousePosMap) <
                            mRoutes[mRouteNow].first().getDistanceTo(mousePosMap)) {
                        mRoutes[mRouteNow].append(mousePosMap);
                        emit routePointAdded(mousePosMap);
                    } else {
                        mRoutes[mRouteNow].prepend(mousePosMap);
                    }
                }
            } else if (e->buttons() & Qt::RightButton) {
                if (routeFound) {
                    mRoutes[mRouteNow].removeAt(routeInd);
                } else {
                    removeLastRoutePoint();
                }
            }
        }
        update();
    } else if (ctrl_shift) {
        if (e->buttons() & Qt::LeftButton) {
            QPoint p = getMousePosRelative();
            llh_t iLlh = mRefLlh;
            xyz_t xyz = {p.x() / 1000.0, p.y() / 1000.0, 0.0};
            llh_t llh = coordinateTransforms::enuToLlh(iLlh, xyz);
            mRefLlh = {llh.latitude, llh.longitude, 0.0};
        }

        update();
    }
}

void MapWidget::mouseReleaseEvent(QMouseEvent *e)
{
    QPoint mousePosWidget = this->mapFromGlobal(QCursor::pos());
    PosPoint mousePosMap;
    QPoint p = getMousePosRelative();
    mousePosMap.setXY(p.x() / 1000.0, p.y() / 1000.0);

    for (const auto& m: mMapModules) {
        if (m->processMouse(false, true, false, false,
                            mousePosWidget, mousePosMap, 0.0,
                            e->modifiers(),
                            e->buttons(),
                            mScaleFactor)) {
            return;
        }
    }

    if (!(e->buttons() & Qt::LeftButton)) {
        mMouseLastX = 1000000;
        mMouseLastY = 1000000;
        mRoutePointSelected = -1;
        mAnchorSelected = -1;
    }
}

void MapWidget::wheelEvent(QWheelEvent *e)
{
    bool ctrl = e->modifiers() == Qt::ControlModifier;

    QPoint mousePosWidget = this->mapFromGlobal(QCursor::pos());
    PosPoint mousePosMap;
    QPoint p = getMousePosRelative();
    mousePosMap.setXY(p.x() / 1000.0, p.y() / 1000.0);

    for (const auto& m: mMapModules) {
        if (m->processMouse(false, false, false, true,
                            mousePosWidget, mousePosMap, e->angleDelta().y(),
                            e->modifiers(),
                            e->buttons(),
                            mScaleFactor)) {
            return;
        }
    }

    if (ctrl && mSelectedObject >= 0) {
		for (int i = 0;i < mObjectStateList.size();i++) {
			QSharedPointer<VehicleState> vehicleState = mObjectStateList[i].dynamicCast<VehicleState>();
            if (vehicleState->getId() == mSelectedObject) {
				PosPoint pos = vehicleState->getPosition();
                double angle = pos.getYaw() + (double)e->angleDelta().y() * 0.0005;
                normalizeAngleRad(angle);
                pos.setYaw(angle);
				vehicleState->setPosition(pos);
                emit posSet(mSelectedObject, pos);
                update();
            }
        }
    } else {
        double scaleDiff = ((double)e->angleDelta().y() / 600.0);
        if (scaleDiff > 0.8)
        {
            scaleDiff = 0.8;
        }

        if (scaleDiff < -0.8)
        {
            scaleDiff = -0.8;
        }

        mScaleFactor += mScaleFactor * scaleDiff;
        mXOffset += mXOffset * scaleDiff;
        mYOffset += mYOffset * scaleDiff;

        emit scaleChanged(mScaleFactor);
        emit offsetChanged(mXOffset, mYOffset);
        update();
    }

    updateClosestInfoPoint();
}

bool MapWidget::event(QEvent *event)
{
    if (event->type() == QEvent::Gesture) {
        QGestureEvent *ge = static_cast<QGestureEvent*>(event);

        if (QGesture *pinch = ge->gesture(Qt::PinchGesture)) {
            QPinchGesture *pg = static_cast<QPinchGesture *>(pinch);

            if (pg->changeFlags() & QPinchGesture::ScaleFactorChanged) {
                mScaleFactor *= pg->scaleFactor();
                mXOffset *= pg->scaleFactor();
                mYOffset *= pg->scaleFactor();
                update();
            }

            return true;
        }
    } else if (event->type() == QEvent::KeyPress) {
        // Generate scroll events from up and down arrow keys
        QKeyEvent *ke = static_cast<QKeyEvent*>(event);
        if (ke->key() == Qt::Key_Up) {
            QWheelEvent we(QPointF(0, 0),
                           QPointF(0, 0),
                           QPoint(0, 0),
                           QPoint(0, 120),
                           0, Qt::Vertical, 0,
                           ke->modifiers());
            wheelEvent(&we);
            return true;
        } else if (ke->key() == Qt::Key_Down) {
            QWheelEvent we(QPointF(0, 0),
                           QPointF(0, 0),
                           QPoint(0, 0),
                           QPoint(0, -120),
                           0, Qt::Vertical, 0,
                           ke->modifiers());
            wheelEvent(&we);
            return true;
        }
    }

    return QWidget::event(event);
}

QList<QSharedPointer<ObjectState> > MapWidget::getObjectStateList() const
{
	return mObjectStateList;
}

QList<QSharedPointer<VehicleState> > MapWidget::getVehicleStateList() const
{
	QList<QSharedPointer<VehicleState>> retval;
	for (const auto& obj : mObjectStateList) {
		auto veh = obj.dynamicCast<VehicleState>();
		if (veh != nullptr) {
			retval.append(veh);
		}
	}
	return retval;
}

quint32 MapWidget::getRoutePointAttributes() const
{
    return mRoutePointAttributes;
}

void MapWidget::setRoutePointAttributes(const quint32 &routePointAttributes)
{
    mRoutePointAttributes = routePointAttributes;
}

void MapWidget::addMapModule(QSharedPointer<MapModule> m)
{
    mMapModules.append(m);
}


void MapWidget::removeMapModule(QSharedPointer<MapModule> m)
{
    for (int i = 0;i < mMapModules.size();i++) {
        if (mMapModules.at(i).get() == m.get()) {
            mMapModules.remove(i);
            break;
        }
    }
}

void MapWidget::removeMapModuleLast()
{
    if (!mMapModules.isEmpty()) {
        mMapModules.removeLast();
    }
}

double MapWidget::getCameraImageOpacity() const
{
    return mCameraImageOpacity;
}

void MapWidget::setCameraImageOpacity(double cameraImageOpacity)
{
    mCameraImageOpacity = cameraImageOpacity;
    update();
}

double MapWidget::getCameraImageWidth() const
{
    return mCameraImageWidth;
}

void MapWidget::setCameraImageWidth(double cameraImageWidth)
{
    mCameraImageWidth = cameraImageWidth;
    update();
}

void MapWidget::setLastCameraImage(const QImage &lastCameraImage)
{
    mLastCameraImage = lastCameraImage;

    if (mCameraImageWidth > 0.0001) {
        update();
    }
}

bool MapWidget::getDrawRouteText() const
{
    return mDrawRouteText;
}

void MapWidget::setDrawRouteText(bool drawRouteText)
{
    mDrawRouteText = drawRouteText;
    update();
}

bool MapWidget::getDrawUwbTrace() const
{
    return mDrawUwbTrace;
}

void MapWidget::setDrawUwbTrace(bool drawUwbTrace)
{
    mDrawUwbTrace = drawUwbTrace;
    update();
}

double MapWidget::getTraceMinSpaceGps() const
{
    return mTraceMinSpaceGps;
}

void MapWidget::setTraceMinSpaceGps(double traceMinSpaceGps)
{
    mTraceMinSpaceGps = traceMinSpaceGps;
}

double MapWidget::getTraceMinSpaceObject() const
{
    return mTraceMinSpaceObject;
}

void MapWidget::setTraceMinSpaceObject(double traceMinSpaceObject)
{
    mTraceMinSpaceObject = traceMinSpaceObject;
}

QTime MapWidget::getRoutePointTime() const
{
    return mRoutePointTime;
}

void MapWidget::setRoutePointTime(const QTime &routePointTime)
{
    mRoutePointTime = routePointTime;
}

int MapWidget::getRouteNow() const
{
    return mRouteNow;
}

void MapWidget::setRouteNow(int routeNow)
{
    mRouteNow = routeNow;
    while (mRoutes.size() < (mRouteNow + 1)) {
        QList<PosPoint> l;
        mRoutes.append(l);
    }

    // Clean empty routes
    while (mRouteNow < (mRoutes.size() - 1)) {
        if (mRoutes.last().isEmpty()) {
            mRoutes.removeLast();
        } else {
            break;
        }
    }

    update();
}

int MapWidget::getInfoTraceNow() const
{
    return mInfoTraceNow;
}

void MapWidget::setInfoTraceNow(int infoTraceNow)
{
    int infoTraceOld = mInfoTraceNow;
    mInfoTraceNow = infoTraceNow;

    while (mInfoTraces.size() < (mInfoTraceNow + 1)) {
        QList<PosPoint> l;
        mInfoTraces.append(l);
    }
    update();

    if (infoTraceOld != mInfoTraceNow) {
        emit infoTraceChanged(mInfoTraceNow);
    }
}

void MapWidget::printPdf(QString path, int width, int height)
{
    if (width == 0) {
        width = this->width();
    }

    if (height == 0) {
        height = this->height();
    }

    QPrinter printer(QPrinter::ScreenResolution);
    printer.setOutputFileName(path);
    printer.setOutputFormat(QPrinter::PdfFormat);
    printer.setColorMode(QPrinter::Color);

    printer.printEngine()->setProperty(QPrintEngine::PPK_Creator, "RControlStation");
    printer.printEngine()->setProperty(QPrintEngine::PPK_DocumentName, "Map");

    QPageLayout pageLayout;
    pageLayout.setMode(QPageLayout::FullPageMode);
    pageLayout.setOrientation(QPageLayout::Portrait);
    pageLayout.setMargins(QMarginsF(0, 0, 0, 0));
    pageLayout.setPageSize(QPageSize(QSize(width, height),
                                     QPageSize::Point, QString(),
                                     QPageSize::ExactMatch));
    printer.setPageLayout(pageLayout);

    QPainter painter(&printer);
    paint(painter, printer.pageRect().width(), printer.pageRect().height(), true);
}

void MapWidget::printPng(QString path, int width, int height)
{
    if (width == 0) {
        width = this->width();
    }

    if (height == 0) {
        height = this->height();
    }

    QImage img(width, height, QImage::Format_ARGB32);
    QPainter painter(&img);
    paint(painter, width, height, true);
    img.save(path, "PNG");
}

bool MapWidget::getDrawOsmStats() const
{
    return mDrawOsmStats;
}

void MapWidget::setDrawOsmStats(bool drawOsmStats)
{
    mDrawOsmStats = drawOsmStats;
    update();
}

bool MapWidget::getDrawGrid() const
{
    return mDrawGrid;
}

void MapWidget::setDrawGrid(bool drawGrid)
{
    bool drawGridOld = mDrawGrid;
    mDrawGrid = drawGrid;

    if (drawGridOld != mDrawGrid) {
        update();
    }
}

void MapWidget::updateClosestInfoPoint()
{
    QPointF mpq = getMousePosRelative();
    PosPoint mp(mpq.x() / 1000.0, mpq.y() / 1000.0);
    double dist_min = 1e30;
    PosPoint closest;

    for (int i = 0;i < mVisibleInfoTracePoints.size();i++) {
        const PosPoint &ip = mVisibleInfoTracePoints[i];
        if (mp.getDistanceTo(ip) < dist_min) {
            dist_min = mp.getDistanceTo(ip);
            closest = ip;
        }
    }

    bool drawBefore = mClosestInfo.getInfo().size() > 0;
    bool drawNow = false;
    if ((dist_min * mScaleFactor) < 0.02) {
        if (closest != mClosestInfo) {
            mClosestInfo = closest;
            update();
        }

        drawNow = true;
    } else {
        mClosestInfo.setInfo("");
    }

    if (drawBefore && !drawNow) {
        update();
    }
}

int MapWidget::drawInfoPoints(QPainter &painter, const QList<PosPoint> &pts,
                              QTransform drawTrans, QTransform txtTrans,
                              const QRectF& viewRect_mm,
                              double min_dist)
{
    int last_visible = 0;
    int drawn = 0;
    QPointF pt_txt;
    QRectF rect_txt;

    painter.setTransform(txtTrans);

    for (int i = 0;i < pts.size();i++) {
        const PosPoint &ip = pts[i];
        QPointF p = ip.getPointMm();
        QPointF p2 = drawTrans.map(p);

        if (viewRect_mm.contains(p)) {
            if (drawn > 0) {
                double dist_view = pts.at(i).getDistanceTo(pts.at(last_visible)) * mScaleFactor;
                if (dist_view < min_dist) {
                    continue;
                }

                last_visible = i;
            }

//            painter.setBrush(ip.getColor());
//            painter.setPen(ip.getColor());
            painter.drawEllipse(p2, ip.getRadius(), ip.getRadius());

            drawn++;
            mVisibleInfoTracePoints.append(ip);

            if (mScaleFactor > mInfoTraceTextZoom) {
                pt_txt.setX(p.x() + 5 / mScaleFactor);
                pt_txt.setY(p.y());
                pt_txt = drawTrans.map(pt_txt);
                painter.setPen(Qt::black);
                painter.setFont(QFont("monospace"));
                rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 20,
                                   pt_txt.x() + 500, pt_txt.y() + 500);
                painter.drawText(rect_txt, Qt::AlignTop | Qt::AlignLeft, ip.getInfo());
            }
        }
    }

    return drawn;
}

int MapWidget::getClosestPoint(PosPoint p, QList<PosPoint> points, double &dist)
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

void MapWidget::drawCircleFast(QPainter &painter, QPointF center, double radius, int type)
{
    painter.drawPixmap(center.x() - radius, center.y() - radius,
                       2.0 * radius, 2.0 * radius, mPixmaps.at(type));
}

int MapWidget::getOsmZoomLevel() const
{
    return mOsmZoomLevel;
}

int MapWidget::getOsmMaxZoomLevel() const
{
    return mOsmMaxZoomLevel;
}

void MapWidget::setOsmMaxZoomLevel(int osmMaxZoomLevel)
{
    mOsmMaxZoomLevel = osmMaxZoomLevel;
    update();
}

double MapWidget::getInfoTraceTextZoom() const
{
    return mInfoTraceTextZoom;
}

void MapWidget::setInfoTraceTextZoom(double infoTraceTextZoom)
{
    mInfoTraceTextZoom = infoTraceTextZoom;
    update();
}

OsmClient *MapWidget::osmClient()
{
    return mOsm;
}

int MapWidget::getInfoTraceNum()
{
    return mInfoTraces.size();
}

int MapWidget::getInfoPointsInTrace(int trace)
{
    int res = -1;

    if (trace >= 0 && trace < mInfoTraces.size()) {
        res = mInfoTraces.at(trace).size();
    }

    return res;
}

int MapWidget::setNextEmptyOrCreateNewInfoTrace()
{
    int next = -1;

    for (int i = 0;i < mInfoTraces.size();i++) {
        if (mInfoTraces.at(i).isEmpty()) {
            next = i;
            break;
        }
    }

    if (next < 0) {
        next = mInfoTraces.size();
    }

    setInfoTraceNow(next);

    return next;
}

void MapWidget::setAnchorMode(bool anchorMode)
{
    mAnchorMode = anchorMode;
}

bool MapWidget::getAnchorMode()
{
    return mAnchorMode;
}

void MapWidget::setAnchorId(int id)
{
    mAnchorId = id;
}

void MapWidget::setAnchorHeight(double height)
{
    mAnchorHeight = height;
}

void MapWidget::removeLastRoutePoint()
{
    PosPoint pos;
    if (mRoutes[mRouteNow].size() > 0) {
        pos = mRoutes[mRouteNow].last();
        mRoutes[mRouteNow].removeLast();
    }
    emit lastRoutePointRemoved(pos);
    update();
}

void MapWidget::zoomInOnRoute(int id, double margins, double wWidth, double wHeight)
{
    QList<PosPoint> route;

    if (id >= 0) {
        route = getRoute(id);
    } else {
        for (auto r: mRoutes) {
            route.append(r);
        }
    }

    if (route.size() > 0) {
        double xMin = 1e12;
        double xMax = -1e12;
        double yMin = 1e12;
        double yMax = -1e12;

        for (auto p: route) {
            if (p.getX() < xMin) {
                xMin = p.getX();
            }
            if (p.getX() > xMax) {
                xMax = p.getX();
            }
            if (p.getY() < yMin) {
                yMin = p.getY();
            }
            if (p.getY() > yMax) {
                yMax = p.getY();
            }
        }

        double width = xMax - xMin;
        double height = yMax - yMin;

        if (wWidth <= 0 || wHeight <= 0) {
            wWidth = this->width();
            wHeight = this->height();
        }

        xMax += width * margins * 0.5;
        xMin -= width * margins * 0.5;
        yMax += height * margins * 0.5;
        yMin -= height * margins * 0.5;

        width = xMax - xMin;
        height = yMax - yMin;

        double scaleX = 1.0 / ((width * 1000) / wWidth);
        double scaleY = 1.0 / ((height * 1000) / wHeight);

        mScaleFactor = qMin(scaleX, scaleY);
        mXOffset = -(xMin + width / 2.0) * mScaleFactor * 1000.0;
        mYOffset = -(yMin + height / 2.0) * mScaleFactor * 1000.0;

        update();
    }
}

double MapWidget::getOsmRes() const
{
    return mOsmRes;
}

void MapWidget::setOsmRes(double osmRes)
{
    mOsmRes = osmRes;
    update();
}

bool MapWidget::getDrawOpenStreetmap() const
{
    return mDrawOpenStreetmap;
}

void MapWidget::setDrawOpenStreetmap(bool drawOpenStreetmap)
{
    mDrawOpenStreetmap = drawOpenStreetmap;
    update();
}

void MapWidget::setEnuRef(const llh_t &llh)
{
    mRefLlh = llh;
    update();
}

llh_t MapWidget::getEnuRef()
{
    return mRefLlh;
}

llh_t const * MapWidget::getEnuRef_Ptr()
{
    return &mRefLlh;
}

double MapWidget::drawGrid(QPainter& painter, QTransform drawTrans, QTransform txtTrans, double gridWidth, double gridHeight)
{
    QPen pen;
    QString gridLabel;
    QPointF gridLabelPos;
    painter.setTransform(txtTrans);

    // Grid parameters
    double stepGrid = 20.0 * ceil(1.0 / ((mScaleFactor * 10.0) / 50.0));
    bool gridDiv = false;
    if (round(log10(stepGrid)) > log10(stepGrid)) {
        gridDiv = true;
    }
    stepGrid = pow(10.0, round(log10(stepGrid)));
    if (gridDiv) {
        stepGrid /= 2.0;
    }
    const double zeroAxisWidth = 3;
    const QColor zeroAxisColor = Qt::red;
    const QColor firstAxisColor = Qt::gray;
    const QColor secondAxisColor = Qt::blue;
    const QColor textColor = QPalette::Foreground;

    // Grid boundries in mm
    const double xStart = -ceil(gridWidth / stepGrid / mScaleFactor) * stepGrid - ceil(mXOffset / stepGrid / mScaleFactor) * stepGrid;
    const double xEnd = ceil(gridWidth / stepGrid / mScaleFactor) * stepGrid - floor(mXOffset / stepGrid / mScaleFactor) * stepGrid;
    const double yStart = -ceil(gridWidth / stepGrid / mScaleFactor) * stepGrid - ceil(mYOffset / stepGrid / mScaleFactor) * stepGrid;
    const double yEnd = ceil(gridWidth / stepGrid / mScaleFactor) * stepGrid - floor(mYOffset / stepGrid / mScaleFactor) * stepGrid;

    // Draw Y-axis segments
    for (double i = xStart;i < xEnd;i += stepGrid) {
        if (fabs(i) < 1e-3) {
            i = 0.0;
        }

        if ((int)(i / stepGrid) % 2) {
            pen.setWidth(0);
            pen.setColor(firstAxisColor);
            painter.setPen(pen);
        } else {
            gridLabel.sprintf("%.2f m", i / 1000.0);

            gridLabelPos.setX(i);
            gridLabelPos.setY(0);
            gridLabelPos = drawTrans.map(gridLabelPos);
            gridLabelPos.setX(gridLabelPos.x() - 5);
            gridLabelPos.setY(gridHeight - 10);
            painter.setPen(QPen(textColor));
            painter.save();
            painter.translate(gridLabelPos);
            painter.rotate(-90);
            painter.drawText(0, 0, gridLabel);
            painter.restore();

            if (fabs(i) < 1e-3) {
                pen.setWidthF(zeroAxisWidth);
                pen.setColor(zeroAxisColor);
            } else {
                pen.setWidth(0);
                pen.setColor(secondAxisColor);
            }
            painter.setPen(pen);
        }

        QPointF pt_start(i, yStart);
        QPointF pt_end(i, yEnd);
        pt_start = drawTrans.map(pt_start);
        pt_end = drawTrans.map(pt_end);
        painter.drawLine(pt_start, pt_end);
    }

    // Draw X-axis segments
    for (double i = yStart;i < yEnd;i += stepGrid) {
        if (fabs(i) < 1e-3) {
            i = 0.0;
        }

        if ((int)(i / stepGrid) % 2) {
            pen.setWidth(0);
            pen.setColor(firstAxisColor);
            painter.setPen(pen);
        } else {
            gridLabel.sprintf("%.2f m", i / 1000.0);
            gridLabelPos.setY(i);

            gridLabelPos = drawTrans.map(gridLabelPos);
            gridLabelPos.setX(10);
            gridLabelPos.setY(gridLabelPos.y() - 5);
            painter.setPen(QPen(textColor));
            painter.drawText(gridLabelPos, gridLabel);

            if (fabs(i) < 1e-3) {
                pen.setWidthF(zeroAxisWidth);
                pen.setColor(zeroAxisColor);
            } else {
                pen.setWidth(0);
                pen.setColor(secondAxisColor);
            }
            painter.setPen(pen);
        }

        QPointF pt_start(xStart, i);
        QPointF pt_end(xEnd, i);
        pt_start = drawTrans.map(pt_start);
        pt_end = drawTrans.map(pt_end);
        painter.drawLine(pt_start, pt_end);
    }

    return stepGrid;
}

QPair<int, int> MapWidget::drawInfoTraces(QPainter& painter, QTransform drawTrans, QTransform txtTrans, const QRectF& viewRect_mm)
{
    int info_segments = 0;
    int info_points = 0;
    QPen pen = painter.pen();

    for (int in = 0;in < mInfoTraces.size();in++) {
        QList<PosPoint> &itNow = mInfoTraces[in];

        if (mInfoTraceNow == in) {
            pen.setColor(Qt::darkGreen);
            painter.setBrush(Qt::green);
        } else {
            pen.setColor(Qt::darkGreen);
            painter.setBrush(Qt::green);
        }

        pen.setWidthF(3.0);
        painter.setPen(pen);
        painter.setTransform(txtTrans);

        const double info_min_dist = 0.02;

        int last_visible = 0;
        for (int i = 1;i < itNow.size();i++) {
            double dist_view = itNow.at(i).getDistanceTo(itNow.at(last_visible)) * mScaleFactor;
            if (dist_view < info_min_dist) {
                continue;
            }

            bool draw = viewRect_mm.contains(itNow[last_visible].getPointMm());

            if (!draw) {
                draw = viewRect_mm.contains(itNow[i].getPointMm());
            }

            if (!draw) {
                draw = isLineSegmentWithinRect(itNow[last_visible].getPointMm(),
                                               itNow[i].getPointMm(),
                                               viewRect_mm);
            }

            if (draw && itNow[i].getDrawLine()) {
                QPointF p1 = drawTrans.map(itNow[last_visible].getPointMm());
                QPointF p2 = drawTrans.map(itNow[i].getPointMm());

                painter.drawLine(p1, p2);
                info_segments++;
            }

            last_visible = i;
        }

        QList<PosPoint> pts_green;
        QList<PosPoint> pts_red;
        QList<PosPoint> pts_other;

        for (int i = 0;i < itNow.size();i++) {
            PosPoint ip = itNow[i];

//            if (mInfoTraceNow != in) {
//                ip.setColor(Qt::gray);
//            }

//            if (ip.getColor() == Qt::darkGreen || ip.getColor() == Qt::green) {
//                pts_green.append(ip);
//            } else if (ip.getColor() == Qt::darkRed || ip.getColor() == Qt::red) {
//                pts_red.append(ip);
//            } else {
                pts_other.append(ip);
//            }
        }

//        info_points += drawInfoPoints(painter, pts_green, drawTrans, txtTrans,
//                                      viewRect_mm, info_min_dist);
        info_points += drawInfoPoints(painter, pts_other, drawTrans, txtTrans,
                                      viewRect_mm, info_min_dist);
//        info_points += drawInfoPoints(painter, pts_red, drawTrans, txtTrans,
//                                      viewRect_mm, info_min_dist);
    }

    // Draw point closest to mouse pointer
    if (mClosestInfo.getInfo().size() > 0) {
        QPointF p = mClosestInfo.getPointMm();
        QPointF p2 = drawTrans.map(p);
        QPointF textPos;
        QRectF textRectangle;

        painter.setTransform(txtTrans);
        pen.setColor(Qt::green);
        painter.setBrush(Qt::green);
        painter.setPen(Qt::green);
        painter.drawEllipse(p2, mClosestInfo.getRadius(), mClosestInfo.getRadius());

        textPos.setX(p.x() + 5 / mScaleFactor);
        textPos.setY(p.y());
        painter.setTransform(txtTrans);
        textPos = drawTrans.map(textPos);
        pen.setColor(Qt::black);
        painter.setPen(pen);
        textRectangle.setCoords(textPos.x(), textPos.y() - 20,
                           textPos.x() + 500, textPos.y() + 500);
        painter.drawText(textRectangle, Qt::AlignTop | Qt::AlignLeft, mClosestInfo.getInfo());
    }

    return QPair<int, int>(info_points, info_segments);
}

void MapWidget::drawOSMTiles(QPainter& painter, QTransform drawTrans, double viewWidth, double viewHeight, QPointF viewCenter, bool highQuality)
{
    painter.setTransform(drawTrans);
    if (mDrawOpenStreetmap) {
        const llh_t &iLlh = mRefLlh;

        mOsmZoomLevel = (int)round(log(mScaleFactor * mOsmRes * 100000000.0 *
                                        cos(iLlh.latitude * M_PI / 180.0)) / log(2.0));
        if (mOsmZoomLevel > mOsmMaxZoomLevel) {
            mOsmZoomLevel = mOsmMaxZoomLevel;
        } else if (mOsmZoomLevel < 0) {
            mOsmZoomLevel = 0;
        }

        int xt = OsmTile::long2tilex(iLlh.longitude, mOsmZoomLevel);
        int yt = OsmTile::lat2tiley(iLlh.latitude, mOsmZoomLevel);

        llh_t llhTile = {OsmTile::tiley2lat(yt, mOsmZoomLevel), OsmTile::tilex2long(xt, mOsmZoomLevel), 0.0};

        xyz_t xyz = coordinateTransforms::llhToEnu(iLlh, llhTile);

        // Calculate scale at ENU origin
        double w = OsmTile::lat2width(iLlh.latitude, mOsmZoomLevel);

        int t_ofs_x = (int)ceil(-(viewCenter.x() - viewWidth / 2.0) / w);
        int t_ofs_y = (int)ceil((viewCenter.y() + viewHeight / 2.0) / w);

        if (!highQuality) {
            painter.setRenderHint(QPainter::SmoothPixmapTransform, mAntialiasOsm);
        }

        QTransform trans = painter.transform();
        trans.scale(1, -1);
        painter.setTransform(trans);

        for (int j = 0;j < 40;j++) {
            for (int i = 0;i < 40;i++) {
                int xt_i = xt + i - t_ofs_x;
                int yt_i = yt + j - t_ofs_y;
                double ts_x = xyz.x + w * i - (double)t_ofs_x * w;
                double ts_y = -xyz.y + w * j - (double)t_ofs_y * w;

                // We are outside the view
                if (ts_x > (viewCenter.x() + viewWidth / 2.0)) {
                    break;
                } else if ((ts_y - w) > (-viewCenter.y() + viewHeight / 2.0)) {
                    break;
                }

                int res;
                OsmTile t = mOsm->getTile(mOsmZoomLevel, xt_i, yt_i, res);

                if (w < 0.0) {
                    w = t.getWidthTop();
                }

                painter.drawPixmap(ts_x * 1000.0, ts_y * 1000.0,
                                   w * 1000.0, w * 1000.0, t.pixmap());

                if (res == 0 && !mOsm->downloadQueueFull()) {
                    mOsm->downloadTile(mOsmZoomLevel, xt_i, yt_i);
                }
            }
        }

        if (!highQuality) {
            painter.setRenderHint(QPainter::SmoothPixmapTransform, mAntialiasDrawings);
        }
    }
}

void MapWidget::drawRoute(QPainter& painter, QTransform drawTrans, QTransform txtTrans, bool highQuality, double scaleFactor, const QList<PosPoint> &route, int routeID, bool isSelected, bool drawAnnotations)
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
            drawCircleFast(painter, p, 10.0 / scaleFactor, isSelected ? 0 : 1); // TODO generalize / refactor
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

void MapWidget::drawAnchor(QPainter &painter, QTransform drawTrans, QTransform txtTrans, const PosPoint &anchor)
{
    QString anchorLabel;
    QPointF anchorLabelPos;
    QRectF anchorLabelRectangle;
    double x, y;
    x = anchor.getX() * 1000.0;
    y = anchor.getY() * 1000.0;
    painter.setTransform(drawTrans);

    // Draw anchor
    painter.setBrush(QBrush(Qt::red));
    painter.translate(x, y);

    anchorLabelPos.setX(x);
    anchorLabelPos.setY(y);
    painter.setTransform(txtTrans);
    anchorLabelPos = drawTrans.map(anchorLabelPos);

    painter.drawRoundedRect(anchorLabelPos.x() - 10,
                            anchorLabelPos.y() - 10,
                            20, 20, 10, 10);

    // Print data
    anchorLabel.sprintf("Anchor %d\n"
                "Pos    : (%.3f, %.3f)\n"
                "Height : %.2f m",
                anchor.getId(),
                anchor.getX(), anchor.getY(),
                anchor.getHeight());
    anchorLabelPos.setX(x);
    anchorLabelPos.setY(y);
    painter.setTransform(txtTrans);
    anchorLabelPos = drawTrans.map(anchorLabelPos);
    anchorLabelRectangle.setCoords(anchorLabelPos.x() + 15, anchorLabelPos.y() - 20,
                       anchorLabelPos.x() + 500, anchorLabelPos.y() + 500);
    painter.drawText(anchorLabelRectangle, anchorLabel);
}

void MapWidget::drawInfoOverlay(QPainter &painter, QTransform txtTrans, double width, double gridResolution, QPair<int, int> infoTraceStats)
{
    double start_txt = 30.0;
    const double txt_row_h = 20.0;
    const double txtOffset = 185.0;

    painter.setTransform(txtTrans);

    if (!mLastCameraImage.isNull() && mCameraImageWidth > 0.001) {
        double imgWidth = (double)width * mCameraImageWidth;
        double imgHeight = (double)mLastCameraImage.height() *
                (imgWidth / (double)mLastCameraImage.width());

        QRectF target(width - imgWidth, 0.0, imgWidth, imgHeight);
        QRectF source(0.0, 0.0, mLastCameraImage.width(), mLastCameraImage.height());

        start_txt += imgHeight;

        painter.setOpacity(mCameraImageOpacity);
        painter.drawImage(target, mLastCameraImage, source);
        painter.setOpacity(1.0);
    }

    QFont font = painter.font();
    painter.setTransform(txtTrans);
    font.setPointSize(10);
    painter.setFont(font);
    painter.setPen(QPen(QPalette::Foreground));
    QString mapAnnotations;

    // Draw units (m)
    if (mDrawGrid) {
        double res = gridResolution / 1000.0;
        if (res >= 1000.0) {
            mapAnnotations.sprintf("Grid res: %.0f km", res / 1000.0);
        } else if (res >= 1.0) {
            mapAnnotations.sprintf("Grid res: %.0f m", res);
        } else {
            mapAnnotations.sprintf("Grid res: %.0f cm", res * 100.0);
        }
        painter.drawText(width - txtOffset, start_txt, mapAnnotations);
        start_txt += txt_row_h;
    }

    // Draw zoom level
    mapAnnotations.sprintf("Zoom: %.7f", mScaleFactor);
    painter.drawText(width - txtOffset, start_txt, mapAnnotations);
    start_txt += txt_row_h;

    // Draw OSM zoom level
    if (mDrawOpenStreetmap) {
        mapAnnotations.sprintf("OSM zoom: %d", mOsmZoomLevel);
        painter.drawText(width - txtOffset, start_txt, mapAnnotations);
        start_txt += txt_row_h;

        if (mDrawOsmStats) {
            mapAnnotations.sprintf("DL Tiles: %d", mOsm->getTilesDownloaded());
            painter.drawText(width - txtOffset, start_txt, mapAnnotations);
            start_txt += txt_row_h;

            mapAnnotations.sprintf("HDD Tiles: %d", mOsm->getHddTilesLoaded());
            painter.drawText(width - txtOffset, start_txt, mapAnnotations);
            start_txt += txt_row_h;

            mapAnnotations.sprintf("RAM Tiles: %d", mOsm->getRamTilesLoaded());
            painter.drawText(width - txtOffset, start_txt, mapAnnotations);
            start_txt += txt_row_h;
        }
    }

    int numInfoPointsInView = infoTraceStats.first;
    int numInfoSegmentsInView = infoTraceStats.second;
    if (numInfoSegmentsInView > 0) {
        mapAnnotations.sprintf("Info seg: %d", numInfoSegmentsInView);
        painter.drawText(width - txtOffset, start_txt, mapAnnotations);
        start_txt += txt_row_h;
    }

    if (numInfoPointsInView > 0) {
        mapAnnotations.sprintf("Info pts: %d", numInfoPointsInView);
        painter.drawText(width - txtOffset, start_txt, mapAnnotations);
        start_txt += txt_row_h;
    }

    // Route info
    if (mRoutes.at(mRouteNow).size() > 0) {
        PosPoint prev = mRoutes.at(mRouteNow).first();
        double len = 0.0;
        for (int i = 1;i < mRoutes.at(mRouteNow).size();i++) {
            len += prev.getDistanceTo(mRoutes.at(mRouteNow).at(i));
            prev = mRoutes.at(mRouteNow).at(i);
        }

        mapAnnotations.sprintf("RP: %d", mRoutes.at(mRouteNow).size());
        painter.drawText(width - txtOffset, start_txt, mapAnnotations);
        start_txt += txt_row_h;

        mapAnnotations.sprintf("RLen: %.2f m", len);
        painter.drawText(width - txtOffset, start_txt, mapAnnotations);
        start_txt += txt_row_h;
    }
}

void MapWidget::paint(QPainter &painter, int width, int height, bool highQuality)
{
    if (highQuality) {
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setRenderHint(QPainter::TextAntialiasing, true);
        painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    } else {
        painter.setRenderHint(QPainter::Antialiasing, mAntialiasDrawings);
        painter.setRenderHint(QPainter::TextAntialiasing, mAntialiasDrawings);
        painter.setRenderHint(QPainter::SmoothPixmapTransform, mAntialiasDrawings);
    }

    constexpr double scaleMax = 20;
    constexpr double scaleMin = 0.000001;

    // Make sure scale and offsetappend is reasonable
    if (mScaleFactor < scaleMin) {
        double scaleDiff = scaleMin / mScaleFactor;
        mScaleFactor = scaleMin;
        mXOffset *= scaleDiff;
        mYOffset *= scaleDiff;
    } else if (mScaleFactor > scaleMax) {
        double scaleDiff = scaleMax / mScaleFactor;
        mScaleFactor = scaleMax;
        mXOffset *= scaleDiff;
        mYOffset *= scaleDiff;
    }

    // Optionally follow a vehicle
    if (mFollowObjectId >= 0) {
		for (int i = 0;i < mObjectStateList.size();i++) {
			QSharedPointer<VehicleState> vehicleState = mObjectStateList[i].dynamicCast<VehicleState>();
            if (vehicleState->getId() == mFollowObjectId) {
				PosPoint followLoc = vehicleState->getPosition();
                mXOffset = -followLoc.getX() * 1000.0 * mScaleFactor;
                mYOffset = -followLoc.getY() * 1000.0 * mScaleFactor;
            }
        }
    }

    // Limit the offset to avoid overflow at 2^31 mm
    double lim = 2000000000.0 * mScaleFactor;
    if (mXOffset > lim) {
        mXOffset = lim;
    } else if (mXOffset < -lim) {
        mXOffset = -lim;
    }

    if (mYOffset > lim) {
        mYOffset = lim;
    } else if (mYOffset < -lim) {
        mYOffset = -lim;
    }

    // Paint begins here
    QPen pen;
    painter.fillRect(0, 0, width, height, QBrush(Qt::transparent));

    // Map coordinate transforms
    QTransform drawTrans;
    drawTrans.translate(width / 2 + mXOffset, height / 2 - mYOffset);
    drawTrans.scale(mScaleFactor, -mScaleFactor);
    drawTrans.rotate(mRotation);

    // Text coordinates
    QTransform txtTrans;
    txtTrans.translate(0, 0);
    txtTrans.scale(1, 1);
    txtTrans.rotate(0);

    // Set font
    QFont font = this->font();
    font.setPointSize(10);
    font.setFamily("Monospace");
    painter.setFont(font);

    // View center, width and height in m
    const QPointF viewCenter(-mXOffset / mScaleFactor / 1000.0, -mYOffset / mScaleFactor / 1000.0);
    const double viewWidth = width / mScaleFactor / 1000.0;
    const double viewHeight = height / mScaleFactor / 1000.0;

    // View boundries in mm
    const QRectF viewRect_mm(QPointF((viewCenter.x() - viewWidth / 2.0) * 1000.0, (viewCenter.y() - viewHeight / 2.0) * 1000.0),
                             QPointF((viewCenter.x() + viewWidth / 2.0) * 1000.0, (viewCenter.y() + viewHeight / 2.0) * 1000.0));

    painter.save();
    // Draw openstreetmap tiles
    drawOSMTiles(painter, drawTrans, viewWidth, viewHeight, viewCenter, highQuality);
    painter.restore();

    double gridResolution;
    if (mDrawGrid)
        gridResolution = drawGrid(painter, drawTrans, txtTrans, width, height);

    // Draw info traces
    QPair<int, int> infoTraceStats = drawInfoTraces(painter, drawTrans, txtTrans, viewRect_mm);

    // Draw trace for the selected vehicle
    pen.setWidthF(5.0 / mScaleFactor);
    pen.setColor(Qt::red);
    painter.setPen(pen);
    painter.setTransform(drawTrans);
    for (int i = 1;i < mObjectTrace.size();i++) {
        painter.drawLine(mObjectTrace[i - 1].getX() * 1000.0, mObjectTrace[i - 1].getY() * 1000.0,
                mObjectTrace[i].getX() * 1000.0, mObjectTrace[i].getY() * 1000.0);
    }

    // Draw GNSS trace for the selected vehicle
    pen.setWidthF(2.5 / mScaleFactor);
    pen.setColor(Qt::magenta);
    painter.setPen(pen);
    painter.setTransform(drawTrans);
    for (int i = 1;i < mObjectTraceGNSS.size();i++) {
        painter.drawLine(mObjectTraceGNSS[i - 1].getX() * 1000.0, mObjectTraceGNSS[i - 1].getY() * 1000.0,
                mObjectTraceGNSS[i].getX() * 1000.0, mObjectTraceGNSS[i].getY() * 1000.0);
    }

    // Draw UWB trace for the selected vehicle
    if (mDrawUwbTrace) {
        pen.setWidthF(2.5 / mScaleFactor);
        pen.setColor(Qt::green);
        painter.setPen(pen);
        painter.setTransform(drawTrans);
        for (int i = 1;i < mObjectTraceUwb.size();i++) {
            painter.drawLine(mObjectTraceUwb[i - 1].getX() * 1000.0, mObjectTraceUwb[i - 1].getY() * 1000.0,
                    mObjectTraceUwb[i].getX() * 1000.0, mObjectTraceUwb[i].getY() * 1000.0);
        }
    }

    // Draw routes
    for (int rn = 0;rn < mRoutes.size();rn++) {
        drawRoute(painter, drawTrans, txtTrans, highQuality, mScaleFactor, mRoutes[rn], rn, rn == mRouteNow, mDrawRouteText);
    }

    // Map module painting
    painter.save();
    painter.setPen(QPen(QPalette::Foreground));
    for (const auto& m: mMapModules) {
        m->processPaint(painter, width, height, highQuality,
                        drawTrans, txtTrans, mScaleFactor);
    }
    painter.restore();

    // Draw vehicles
    painter.setPen(QPen(QPalette::Foreground));
	for(const auto& obj : mObjectStateList)
        obj->draw(painter, drawTrans, txtTrans, obj->getId() == mSelectedObject);

    painter.setPen(QPen(QPalette::Foreground));

    // Draw anchors
    for (const auto& anchor : mAnchors) {
        drawAnchor(painter, drawTrans, txtTrans, anchor);
    }

    drawInfoOverlay(painter, txtTrans, width, gridResolution, infoTraceStats);

    painter.end();
}

void MapWidget::updateTraces()
{
    // Store trace for the selected object
    if (mTraceObject >= 0) {
		for (int i = 0;i < mObjectStateList.size();i++) {
			QSharedPointer<ObjectState> objectState = mObjectStateList[i];
            if (objectState->getId() == mTraceObject) {
                if (mObjectTrace.isEmpty()) {
                    mObjectTrace.append(objectState->getPosition());
                }
                if (mObjectTrace.last().getDistanceTo(objectState->getPosition()) > mTraceMinSpaceObject) {
                    mObjectTrace.append(objectState->getPosition());
                }
//                // GPS trace
//                if (mVehicleTraceGps.isEmpty()) {
//                    mVehicleTraceGps.append(VehicleState->getLocationGps());
//                }
//                if (mVehicleTraceGps.last().getDistanceTo(VehicleState->getLocationGps()) > mTraceMinSpaceGps) {
//                    mVehicleTraceGps.append(VehicleState->getLocationGps());
//                }
//                // UWB trace
//                if (mVehicleTraceUwb.isEmpty()) {
//                    mVehicleTraceUwb.append(VehicleState->getLocationUwb());
//                }
//                if (mVehicleTraceUwb.last().getDistanceTo(VehicleState->getLocationUwb()) > mTraceMinSpaceVehicle) {
//                    mVehicleTraceUwb.append(VehicleState->getLocationUwb());
//                }
            }
        }
    }

    // Truncate traces
    while (mObjectTrace.size() > 5000) {
        mObjectTrace.removeFirst();
    }

    while (mObjectTraceGNSS.size() > 1800) {
        mObjectTraceGNSS.removeFirst();
    }

    while (mObjectTraceUwb.size() > 5000) {
        mObjectTraceUwb.removeFirst();
    }
}
