/*
 *     Copyright 2016 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
    */

#include "osmtile.h"
#include <cmath>

OsmTile::OsmTile(QPixmap pxm, int zoom, int x, int y) : mPixmap(pxm), mZoom(zoom), mX(x), mY(y)
{

}

OsmTile::OsmTile(int zoom, int x, int y) : mZoom(zoom), mX(x), mY(y)
{

}

void OsmTile::setZXY(int zoom, int x, int y)
{
    mZoom = zoom;
    mX = x;
    mY = y;
}

int OsmTile::zoom() const
{
    return mZoom;
}

void OsmTile::setZoom(double zoom)
{
    mZoom = zoom;
}

int OsmTile::x() const
{
    return mX;
}

void OsmTile::setX(int x)
{
    mX = x;
}

int OsmTile::y() const
{
    return mY;
}

void OsmTile::setY(int y)
{
    mY = y;
}

QPixmap OsmTile::pixmap() const
{
    return mPixmap;
}

void OsmTile::setPixmap(const QPixmap &pixmap)
{
    mPixmap = pixmap;
}

double OsmTile::getNorth()
{
    return tiley2lat(mY, mZoom);
}

double OsmTile::getSouth()
{
    return tiley2lat(mY + 1, mZoom);
}

double OsmTile::getWest()
{
    return tilex2long(mX, mZoom);
}

double OsmTile::getEast()
{
    return tilex2long(mX + 1, mZoom);
}

double OsmTile::getWidthTop()
{
    return lat2width(getNorth(), mZoom);
}

int OsmTile::long2tilex(double lon, int z)
{
    return (int)(floor((lon + 180.0) / 360.0 * pow(2.0, z)));
}

int OsmTile::lat2tiley(double lat, int z)
{
    return (int)(floor((1.0 - log(tan(lat * M_PI/180.0) + 1.0 / cos(lat * M_PI/180.0)) / M_PI) / 2.0 * pow(2.0, z)));
}

double OsmTile::tilex2long(int x, int z)
{
    return x / pow(2.0, z) * 360.0 - 180;
}

double OsmTile::tiley2lat(int y, int z)
{
    double n = M_PI - 2.0 * M_PI * y / pow(2.0, z);
    return 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
}

double OsmTile::lat2width(double lat, int zoom)
{
    return 156543.03392 * cos(lat * M_PI / 180.0) / ((double)(1 << zoom)) * 256.0;
}

bool OsmTile::operator==(const OsmTile &tile)
{
    if (mX == tile.mX &&
            mY == tile.mY &&
            mZoom == tile.mZoom) {
        return true;
    } else {
        return false;
    }
}

bool OsmTile::operator!=(const OsmTile &tile)
{
    return !(operator==(tile));
}
