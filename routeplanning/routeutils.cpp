/*
 *     Copyright 2025 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "routeutils.h"

QList<PosPoint> readRouteFromFile(const QString &fileName, llh_t vehicleEnuRef)
{
    QList<PosPoint> importedRoute;

    if (fileName.isEmpty()) {
        qDebug() << "No route file specified.";
        return importedRoute;
    }

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) {
        qDebug() << "Could not open \"" + fileName + "\" for reading.";
        return importedRoute;
    }


    QXmlStreamReader stream(&file);
    if (stream.readNextStartElement())
    {
        if (stream.name() == "routes") {
            llh_t importedEnuRef{0.0, 0.0, 0.0};
            while(stream.readNextStartElement())
            {
                if(stream.name() == "enuref")
                {
                    while(stream.readNextStartElement())
                    {
                        if (stream.name() == "Latitude")
                            importedEnuRef.latitude = stream.readElementText().toDouble();
                        if (stream.name() == "Longitude")
                            importedEnuRef.longitude = stream.readElementText().toDouble();
                        if (stream.name() == "Height")
                            importedEnuRef.height = stream.readElementText().toDouble();
                    }
                }
                if (stream.name() == "route")
                {
                    while(stream.readNextStartElement())
                    {
                        if (stream.name() == "point")
                        {
                            PosPoint importedPoint;

                            while(stream.readNextStartElement())
                            {
                                if (stream.name() == "x")
                                    importedPoint.setX(stream.readElementText().toDouble());
                                if (stream.name() == "y")
                                    importedPoint.setY(stream.readElementText().toDouble());
                                if (stream.name() == "z")
                                    importedPoint.setHeight(stream.readElementText().toDouble());
                                if (stream.name() == "speed")
                                    importedPoint.setSpeed(stream.readElementText().toDouble());
                                if (stream.name() == "attributes")
                                    importedPoint.setAttributes(stream.readElementText().toUInt());
                            }

                            llh_t importedAbsPoint = coordinateTransforms::enuToLlh(importedEnuRef, {importedPoint.getX(), importedPoint.getY(), importedPoint.getHeight()});
                            xyz_t importedEnuPoint = coordinateTransforms::llhToEnu(vehicleEnuRef, importedAbsPoint);

                            importedPoint.setX(importedEnuPoint.x);
                            importedPoint.setY(importedEnuPoint.y);
                            importedPoint.setHeight(importedEnuPoint.z);

                            importedRoute.append(importedPoint);
                        }
                    }
                }
            }
        }
    }
    return importedRoute;
}

bool isPointWithin(double px, double py, QList<PosPoint> route)
{
    if (route.size() < 3) {
        return false;
    }
    int nVert = route.size();
    int i,j;
    bool c = false;

    for (i = 0, j = nVert -  1; i < nVert; j = i ++) {
        double vxi = route.at(i).getX();
        double vyi = route.at(i).getY();
        double vxj = route.at(j).getX();
        double vyj = route.at(j).getY();

        if (((vyi > py) != (vyj > py)) &&
            (px < (vxj-vxi) * (py-vyi) / (vyj-vyi) + vxi)) {
            c = !c;
        }
    }
    return c;
}

bool isPointWithin(PosPoint p, QList<PosPoint> route)
{
    return isPointWithin(p.getX(),p.getY(),route);
}