/*
    Copyright 2012 - 2019 Benjamin Vedder	benjamin@vedder.se
              2020 - 2022 Marvin Damschen  marvin.damschen@ri.se

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

#include <QDebug>
#include <math.h>
#include <qmath.h>
#include <QPrinter>
#include <QPrintEngine>
#include <QTime>
#include <QTextStream>

#include "mapwidget.h"

MapWidget::MapWidget(QWidget *parent) : QWidget(parent)
{
    qRegisterMetaType<llh_t>();

    mScaleFactor = 0.1;
    mRotation = 0;
    mXOffset = 0;
    mYOffset = 0;
    mMouseLastX = 1000000;
    mMouseLastY = 1000000;
    mFollowObjectId = -1;
    mAntialiasDrawings = false;
    mAntialiasOsm = true;
    mDrawGrid = true;

    mOsm = QSharedPointer<OsmClient>::create(this);
    mDrawOpenStreetmap = true;
    mOsmZoomLevel = 15;
    mOsmRes = 1.0;
    mOsmMaxZoomLevel = 19;
    mDrawOsmStats = false;


    // ASTA
    //mRefLlh = {57.78100308, 12.76925422, 253.76};

    // RISE RTK base station
    //mRefLlh = {57.71495867, 12.89134921, 219.0};
    
    mRefLlh = {57.68412443207728, 11.983627080917358, 51};

    // Hardcoded for now
    mOsm->setCacheDir("osm_tiles");
    //    mOsm->setTileServerUrl("http://tile.openstreetmap.org");
    mOsm->setTileServerUrl("http://c.osm.rrze.fau.de/osmhd"); // Also https

    connect(mOsm.get(), &OsmClient::tileReady, this, &MapWidget::tileReady);
    connect(mOsm.get(), &OsmClient::errorGetTile, this, &MapWidget::errorGetTile);

    setMouseTracking(true);
    grabGesture(Qt::PinchGesture);
}

void MapWidget::addObjectState(QSharedPointer<ObjectState> objectState)
{
    mObjectStateMap.insert(objectState->getId(), objectState);
    connect(objectState.get(), &ObjectState::positionUpdated, this, &MapWidget::triggerUpdate);
}

QSharedPointer<ObjectState> MapWidget::getObjectState(int objectID)
{
    return mObjectStateMap.value(objectID);
}

bool MapWidget::removeObjectState(int objectID)
{
    QObject::disconnect(mObjectStateMap.value(objectID).get(), &ObjectState::positionUpdated, this, &MapWidget::triggerUpdate);

    bool removedAnElement = mObjectStateMap.remove(objectID);
    update();

    return removedAnElement;
}

void MapWidget::clearObjectStates()
{
    mObjectStateMap.clear();
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

void MapWidget::triggerUpdate()
{
    update();
}

void MapWidget::executeContextMenu(QMenu &contextMenu)
{
    contextMenu.exec(QCursor::pos());
}

void MapWidget::setSelectedObjectState(int objectID)
{
    int oldObject = mSelectedObject;
    mSelectedObject = objectID;

    if (oldObject != mSelectedObject) {
        update();
    }
}

void MapWidget::setFollowObjectState(int objectID)
{
    int oldObject = mFollowObjectId;
    mFollowObjectId = objectID;

    if (oldObject != mFollowObjectId) {
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

    // Move map
    if (e->buttons() & Qt::LeftButton &&
            (e->modifiers() & Qt::ControlModifier) == 0 && (e->modifiers() & Qt::ShiftModifier) == 0) {
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
}

void MapWidget::mousePressEvent(QMouseEvent *e)
{
    setFocus();

    QPoint mousePosWidget = this->mapFromGlobal(QCursor::pos());
    PosPoint mousePosMap;
    QPoint p = getMousePosRelative();
    mousePosMap.setXY(p.x() / 1000.0, p.y() / 1000.0);

    if (e->buttons() == Qt::RightButton && e->modifiers() == Qt::NoModifier) { // right click -> create context menu
        QMenu rightClickContextMenu;
        for (const auto& m: mMapModules) {
            QSharedPointer<QMenu> mapModuleContextMenu = m->populateContextMenu({mousePosMap.getX(), mousePosMap.getY(), 0.0}, mRefLlh);
            if (!mapModuleContextMenu.isNull())
                rightClickContextMenu.addActions(mapModuleContextMenu->actions());
        }

        if (!rightClickContextMenu.isEmpty())
            executeContextMenu(rightClickContextMenu);
    } else {
        for (const auto& m: mMapModules) {
            if (m->processMouse(true, false, false, false,
                                mousePosWidget, mousePosMap, 0.0,
                                e->modifiers(),
                                e->buttons(),
                                mScaleFactor)) {
                return;
            }
        }
    }

    // ctrl + shift: set new ENU ref
    if (e->modifiers() == (Qt::ControlModifier | Qt::ShiftModifier)) {
        if (e->buttons() & Qt::LeftButton) {
            QPoint p = getMousePosRelative();
            llh_t iLlh = mRefLlh;
            xyz_t xyz = {p.x() / 1000.0, p.y() / 1000.0, 0.0};
            llh_t llh = coordinateTransforms::enuToLlh(iLlh, xyz);
            setEnuRef({llh.latitude, llh.longitude, 0.0});
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
    }
}

void MapWidget::wheelEvent(QWheelEvent *e)
{
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

    // zoom map
    double scaleDiff = ((double)e->angleDelta().y() / 600.0);
    if (scaleDiff > 0.8)
        scaleDiff = 0.8;

    if (scaleDiff < -0.8)
        scaleDiff = -0.8;

    mScaleFactor += mScaleFactor * scaleDiff;
    mXOffset += mXOffset * scaleDiff;
    mYOffset += mYOffset * scaleDiff;

    emit scaleChanged(mScaleFactor);
    emit offsetChanged(mXOffset, mYOffset);
    update();
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
        // Generate scroll events from up / + and down / - arrow keys
        QKeyEvent *ke = static_cast<QKeyEvent*>(event);
        if (ke->key() == Qt::Key_Up || ke->key() == Qt::Key_Plus) {
            QWheelEvent we(QPointF(0, 0),
                           QPointF(0, 0),
                           QPoint(0, 0),
                           QPoint(0, 120),
                           Qt::MouseButton::NoButton,
                           ke->modifiers(),
                           Qt::ScrollPhase::ScrollBegin,
                           false);
            wheelEvent(&we);
            return true;
        } else if (ke->key() == Qt::Key_Down || ke->key() == Qt::Key_Minus) {
            QWheelEvent we(QPointF(0, 0),
                           QPointF(0, 0),
                           QPoint(0, 0),
                           QPoint(0, -120),
                           Qt::MouseButton::NoButton,
                           ke->modifiers(),
                           Qt::ScrollPhase::ScrollBegin,
                           false);
            wheelEvent(&we);
            return true;
        }
    }

    return QWidget::event(event);
}

QList<QSharedPointer<ObjectState> > MapWidget::getObjectStateList() const
{
    return mObjectStateMap.values();
}

void MapWidget::addMapModule(QSharedPointer<MapModule> m)
{
    mMapModules.append(m);
    connect(m.get(), &MapModule::requestRepaint, this, &MapWidget::triggerUpdate);
    connect(m.get(), &MapModule::requestContextMenu, this, &MapWidget::executeContextMenu);
}


void MapWidget::removeMapModule(QSharedPointer<MapModule> m)
{
    for (int i = 0;i < mMapModules.size();i++) {
        if (mMapModules.at(i).get() == m.get()) {
            mMapModules.remove(i);
            disconnect(m.get(), &MapModule::requestRepaint, this, &MapWidget::triggerUpdate);
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

    printer.printEngine()->setProperty(QPrintEngine::PPK_Creator, "ControlTower");
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
    paint(painter, printer.pageLayout().paintRectPixels(printer.resolution()).width(), printer.pageLayout().paintRectPixels(printer.resolution()).height(), true);
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

void MapWidget::setDrawOsmStats(bool drawOsmStats)
{
    mDrawOsmStats = drawOsmStats;
    update();
}

void MapWidget::setDrawGrid(bool drawGrid)
{
    bool drawGridOld = mDrawGrid;
    mDrawGrid = drawGrid;

    if (drawGridOld != mDrawGrid) {
        update();
    }
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

double MapWidget::getOsmRes() const
{
    return mOsmRes;
}

void MapWidget::setOsmRes(double osmRes)
{
    mOsmRes = osmRes;
    update();
}

void MapWidget::setDrawOpenStreetmap(bool drawOpenStreetmap)
{
    mDrawOpenStreetmap = drawOpenStreetmap;
    update();
}

void MapWidget::setEnuRef(const llh_t &llh)
{
    static llh_t lastEnuRef = mRefLlh;

    mRefLlh = llh;
    update();

    if (mRefLlh.latitude != lastEnuRef.latitude
            && mRefLlh.longitude != lastEnuRef.longitude
            && mRefLlh.height != lastEnuRef.height)
        emit enuRefChanged(mRefLlh);
}

llh_t MapWidget::getEnuRef()
{
    return mRefLlh;
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
    const QColor textColor = QPalette::WindowText;

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
            gridLabel = "";
            QTextStream gridLabelStream(&gridLabel);
            gridLabelStream.setRealNumberPrecision(2);
            gridLabelStream << i / 1000.0 << " m";

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
            gridLabel = "";
            QTextStream gridLabelStream(&gridLabel);
            gridLabelStream.setRealNumberPrecision(2);
            gridLabelStream << i / 1000.0 << " m";

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
        for (const auto& objectState : mObjectStateMap)
            if (objectState->getId() == mFollowObjectId) {
                PosPoint followLoc = objectState->getPosition();
                mXOffset = -followLoc.getX() * 1000.0 * mScaleFactor;
                mYOffset = -followLoc.getY() * 1000.0 * mScaleFactor;
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

    if (mDrawGrid)
        drawGrid(painter, drawTrans, txtTrans, width, height);

    // Map module painting
    painter.save();
    painter.setPen(QPen(QPalette::WindowText));
    for (const auto& m: mMapModules) {
        m->processPaint(painter, width, height, highQuality,
                        drawTrans, txtTrans, mScaleFactor);
    }
    painter.restore();

    // Draw vehicles
    painter.setPen(QPen(QPalette::WindowText));
    for(const auto& obj : mObjectStateMap)
        obj->draw(painter, drawTrans, txtTrans, obj->getId() == mSelectedObject);

    painter.setPen(QPen(QPalette::WindowText));

    painter.end();
}
