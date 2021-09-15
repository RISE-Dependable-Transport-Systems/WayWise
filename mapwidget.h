/*
    Copyright 2012 - 2019 Benjamin Vedder	benjamin@vedder.se
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

#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QBrush>
#include <QFont>
#include <QPen>
#include <QPalette>
#include <QList>
#include <QInputDialog>
#include <QTimer>
#include <QPinchGesture>
#include <QImage>
#include <QTransform>

#include "pospoint.h"
#include "vehiclestate.h"
#include "osmclient.h"
#include "coordinatetransforms.h"

class MapModule
{
public:
    virtual void processPaint(QPainter &painter, int width, int height, bool highQuality,
                              QTransform drawTrans, QTransform txtTrans, double scale) = 0;
    virtual bool processMouse(bool isPress, bool isRelease, bool isMove, bool isWheel,
                              QPoint widgetPos, PosPoint mapPos, double wheelAngleDelta,
                              Qt::KeyboardModifiers keyboardModifiers,
                              Qt::MouseButtons mouseButtons, double scale) = 0;
};

class MapWidget : public QWidget
{
    Q_OBJECT

public:
    typedef enum {
        Default,
        Inactive,
        LAST
    } RoutePointType;

    explicit MapWidget(QWidget *parent = 0);
    QSharedPointer<VehicleState> getVehicleState(int vehicleID);
    void setFollowVehicle(int vehicleID);
    void setTraceVehicle(int vehicleID);
    void setSelectedVehicle(int vehicleID);
    void addVehicle(QSharedPointer<VehicleState> vehicleState);
    bool removeVehicle(int vehicleID);
    void clearVehicles();
    PosPoint* getAnchor(int anchorId);
    void addAnchor(const PosPoint &anchor);
    bool removeAnchor(int anchorId);
    void clearAnchors();
    QList<PosPoint> getAnchors();
    void setScaleFactor(double scale);
    double getScaleFactor();
    void setRotation(double rotation);
    void setXOffset(double offset);
    void setYOffset(double offset);
    void moveView(double px, double py);
    void clearTrace();
	void addRoutePoint(double px, double py, double speed = 0.0, QTime time = QTime());
    QList<PosPoint> getRoute(int ind = -1);
    QList<QList<PosPoint> > getRoutes();
    void setRoute(const QList<PosPoint> &route);
    void addRoute(const QList<PosPoint> &route);
    int getRouteNum();
    void clearRoute();
    void clearAllRoutes();
    void setRoutePointSpeed(double speed);
    void addInfoPoint(PosPoint &info);
    void clearInfoTrace();
    void clearAllInfoTraces();
    QPoint getMousePosRelative();
    void setAntialiasDrawings(bool antialias);
    void setAntialiasOsm(bool antialias);
    bool getDrawOpenStreetmap() const;
    void setDrawOpenStreetmap(bool drawOpenStreetmap);
    void setEnuRef(const llh_t &llh);
    llh_t getEnuRef();
    llh_t const *getEnuRef_Ptr();
    double getOsmRes() const;
    void setOsmRes(double osmRes);
    double getInfoTraceTextZoom() const;
    void setInfoTraceTextZoom(double infoTraceTextZoom);
    OsmClient *osmClient();
    int getInfoTraceNum();
    int getInfoPointsInTrace(int trace);
    int setNextEmptyOrCreateNewInfoTrace();
    void setAnchorMode(bool anchorMode);
    bool getAnchorMode();
    void setAnchorId(int id);
    void setAnchorHeight(double height);
    void removeLastRoutePoint();
    void zoomInOnRoute(int id, double margins, double wWidth = -1, double wHeight = -1);

    int getOsmMaxZoomLevel() const;
    void setOsmMaxZoomLevel(int osmMaxZoomLevel);

    int getOsmZoomLevel() const;

    bool getDrawGrid() const;
    void setDrawGrid(bool drawGrid);

    bool getDrawOsmStats() const;
    void setDrawOsmStats(bool drawOsmStats);

    int getRouteNow() const;
    void setRouteNow(int routeNow);

	QTime getRoutePointTime() const;
	void setRoutePointTime(const QTime& routePointTime);

    double getTraceMinSpaceVehicle() const;
    void setTraceMinSpaceVehicle(double traceMinSpaceVehicle);

    double getTraceMinSpaceGps() const;
    void setTraceMinSpaceGps(double traceMinSpaceGps);

    int getInfoTraceNow() const;
    void setInfoTraceNow(int infoTraceNow);

    void printPdf(QString path, int width = 0, int height = 0);
    void printPng(QString path, int width = 0, int height = 0);

    bool getDrawRouteText() const;
    void setDrawRouteText(bool drawRouteText);

    bool getDrawUwbTrace() const;
    void setDrawUwbTrace(bool drawUwbTrace);

    void setLastCameraImage(const QImage &lastCameraImage);

    double getCameraImageWidth() const;
    void setCameraImageWidth(double cameraImageWidth);

    double getCameraImageOpacity() const;
    void setCameraImageOpacity(double cameraImageOpacity);

    void addMapModule(QSharedPointer<MapModule> m);
    void removeMapModule(QSharedPointer<MapModule> m);
    void removeMapModuleLast();

    quint32 getRoutePointAttributes() const;
    void setRoutePointAttributes(const quint32 &routePointAttributes);

    QList<QSharedPointer<VehicleState> > getVehicleStateList() const;

    double drawGrid(QPainter &painter, QTransform drawTrans, QTransform txtTrans, double gridWidth, double gridHeight);
    QPair<int, int> drawInfoTraces(QPainter& painter, QTransform drawTrans, QTransform txtTrans, const QRectF &viewRect_mm);

    void drawOSMTiles(QPainter &painter, QTransform drawTrans, double viewWidth, double viewHeight, QPointF viewCenter, bool highQuality);

    void drawRoute(QPainter &painter, QTransform drawTrans, QTransform txtTrans, bool highQuality, double scaleFactor, const QList<PosPoint> &route, int routeID, bool isSelected, bool drawAnnotations);

    void drawAnchor(QPainter &painter, QTransform drawTrans, QTransform txtTrans, const PosPoint &anchor);

    void drawInfoOverlay(QPainter &painter, QTransform txtTrans, double width, double gridResolution, QPair<int, int> infoTraceStats);

signals:
    void scaleChanged(double newScale);
    void offsetChanged(double newXOffset, double newYOffset);
    void posSet(quint8 id, PosPoint pos);
    void routePointAdded(PosPoint pos);
    void lastRoutePointRemoved(PosPoint pos);
    void infoTraceChanged(int traceNow);

private slots:
    void tileReady(OsmTile tile);
    void errorGetTile(QString reason);
    void timerSlot();
    void vehiclePositionUpdated();

protected:
    void paintEvent(QPaintEvent *event) override;
    void mouseMoveEvent (QMouseEvent *e) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;
    bool event(QEvent *event) override;

private:
    QList<QSharedPointer<VehicleState>> mVehicleStateList;
    QVector<PosPoint> mVehicleTrace;
    QVector<PosPoint> mVehicleTraceGNSS;
    QVector<PosPoint> mVehicleTraceUwb;
    QList<PosPoint> mAnchors;
    QList<QList<PosPoint> > mRoutes;
    QList<QList<PosPoint> > mInfoTraces;
    QList<PosPoint> mVisibleInfoTracePoints;
    double mRoutePointSpeed;
	QTime mRoutePointTime;
    quint32 mRoutePointAttributes;
    qint32 mAnchorId;
    double mAnchorHeight;
    double mScaleFactor;
    double mRotation;
    double mXOffset;
    double mYOffset;
    int mMouseLastX;
    int mMouseLastY;
    int mFollowVehicleId;
    int mTraceVehicle;
    int mSelectedVehicle;
    double xRealPos;
    double yRealPos;
    bool mAntialiasDrawings;
    bool mAntialiasOsm;
    double mOsmRes;
    double mInfoTraceTextZoom;
    OsmClient *mOsm;
    int mOsmZoomLevel;
    int mOsmMaxZoomLevel;
    bool mDrawOpenStreetmap;
    bool mDrawOsmStats;
    llh_t mRefLlh;
    PosPoint mClosestInfo;
    bool mDrawGrid;
    int mRoutePointSelected;
    int mAnchorSelected;
    int mRouteNow;
    int mInfoTraceNow;
    double mTraceMinSpaceVehicle;
    double mTraceMinSpaceGps;
    QList<QPixmap> mPixmaps;
    bool mAnchorMode;
    bool mDrawRouteText;
    bool mDrawUwbTrace;
    QImage mLastCameraImage;
    double mCameraImageWidth;
    double mCameraImageOpacity;
    QTimer *mTimer;
    QVector<QSharedPointer<MapModule>> mMapModules;

    void updateClosestInfoPoint();
    int drawInfoPoints(QPainter &painter, const QList<PosPoint> &pts,
                        QTransform drawTrans, QTransform txtTrans,
                       const QRectF& viewRect_mm,
                       double min_dist);
    int getClosestPoint(PosPoint p, QList<PosPoint> points, double &dist);
    void drawCircleFast(QPainter &painter, QPointF center, double radius, int type = 0);

    void paint(QPainter &painter, int width, int height, bool highQuality = false);
    void updateTraces();
};

#endif // MAPWIDGET_H
