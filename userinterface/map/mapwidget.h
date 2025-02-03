/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
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
#include <QMenu>

#include "core/pospoint.h"
#include "vehicles/vehiclestate.h"
#include "vehicles/objectstate.h"
#include "osmclient.h"
#include "core/coordinatetransforms.h"

Q_DECLARE_METATYPE(llh_t)

class MapModule : public QObject
{
    Q_OBJECT
public:
    virtual void processPaint(QPainter &painter, int width, int height, bool highQuality,
                              QTransform drawTrans, QTransform txtTrans, double scale) {Q_UNUSED(painter) Q_UNUSED(width) Q_UNUSED(height) Q_UNUSED(highQuality) Q_UNUSED(drawTrans) Q_UNUSED(txtTrans) Q_UNUSED(scale)};
    virtual bool processMouse(bool isPress, bool isRelease, bool isMove, bool isWheel,
                              QPoint widgetPos, PosPoint mapPos, double wheelAngleDelta,
                              Qt::KeyboardModifiers keyboardModifiers,
                              Qt::MouseButtons mouseButtons, double scale) {Q_UNUSED(isPress) Q_UNUSED(isRelease) Q_UNUSED(isMove) Q_UNUSED(isWheel) Q_UNUSED(widgetPos) Q_UNUSED(mapPos)
                                                                            Q_UNUSED(wheelAngleDelta) Q_UNUSED(keyboardModifiers) Q_UNUSED(mouseButtons) Q_UNUSED(scale)
                                                                            return false; };
    virtual QSharedPointer<QMenu> populateContextMenu(const xyz_t& mapPos, const llh_t& enuReference) { Q_UNUSED(mapPos) Q_UNUSED(enuReference)
                                                                                                        return nullptr;};

signals:
    void requestRepaint();
    void requestContextMenu(QMenu& contextMenu);
};

class MapWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MapWidget(QWidget *parent = 0);
    void setScaleFactor(double scale);
    double getScaleFactor();
    void setRotation(double rotation);
    void setXOffset(double offset);
    void setYOffset(double offset);
    void moveView(double px, double py);
    QPoint getMousePosRelative();
    void setAntialiasDrawings(bool antialias);

    void setEnuRef(const llh_t &llh);
    llh_t getEnuRef();

    void setAntialiasOsm(bool antialias);
    void setDrawOpenStreetmap(bool drawOpenStreetmap);
    double getOsmRes() const;
    void setOsmRes(double osmRes);
    int getOsmMaxZoomLevel() const;
    void setOsmMaxZoomLevel(int osmMaxZoomLevel);
    int getOsmZoomLevel() const;
    void setDrawGrid(bool drawGrid);
    void setDrawOsmStats(bool drawOsmStats);

    void printPdf(QString path, int width = 0, int height = 0);
    void printPng(QString path, int width = 0, int height = 0);

    void addObjectState(QSharedPointer<ObjectState> objectState);
    QSharedPointer<ObjectState> getObjectState(int objectID);
    void setFollowObjectState(int objectID);
    void setSelectedObjectState(int objectID);
    bool removeObjectState(int objectID);
    void clearObjectStates();

    void addMapModule(QSharedPointer<MapModule> m);
    void removeMapModule(QSharedPointer<MapModule> m);
    void removeMapModuleLast();

    QList<QSharedPointer<ObjectState>> getObjectStateList() const;

    bool setTileServerUrl(QString path);

signals:
    void scaleChanged(double newScale);
    void offsetChanged(double newXOffset, double newYOffset);
    void posSet(quint8 id, PosPoint pos);
    void enuRefChanged(const llh_t &llh);

private slots:
    void tileReady(OsmTile tile);
    void errorGetTile(QString reason);
    void triggerUpdate();
    void executeContextMenu(QMenu &contextMenu);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mouseMoveEvent (QMouseEvent *e) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;
    bool event(QEvent *event) override;

    double drawGrid(QPainter &painter, QTransform drawTrans, QTransform txtTrans, double gridWidth, double gridHeight);
    void drawOSMTiles(QPainter &painter, QTransform drawTrans, double viewWidth, double viewHeight, QPointF viewCenter, bool highQuality);
    void drawInfoOverlay(QPainter &painter, QTransform txtTrans, double width, double gridResolution, QPair<int, int> infoTraceStats);

private:
    QMap<int, QSharedPointer<ObjectState>> mObjectStateMap;
    llh_t mRefLlh;
    double mScaleFactor;
    double mRotation;
    double mXOffset;
    double mYOffset;
    int mMouseLastX;
    int mMouseLastY;
    int mSelectedObject;
    int mFollowObjectId;
    bool mAntialiasDrawings;
    bool mAntialiasOsm;
    double mOsmRes;
    QSharedPointer<OsmClient> mOsm;
    int mOsmZoomLevel;
    int mOsmMaxZoomLevel;
    bool mDrawOpenStreetmap;
    bool mDrawOsmStats;
    bool mDrawGrid;
    QList<QPixmap> mPixmaps;

    QVector<QSharedPointer<MapModule>> mMapModules;

    void paint(QPainter &painter, int width, int height, bool highQuality = false);
};

#endif // MAPWIDGET_H
