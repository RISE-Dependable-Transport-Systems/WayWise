/*
 *     Copyright 2025 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#ifndef ROUTEUTILS_H
#define ROUTEUTILS_H

#include <QList>
#include <QString>
#include <QFile>
#include <QXmlStreamReader>
#include <QDebug>

#include "core/pospoint.h"
#include "core/coordinatetransforms.h"


QList<PosPoint> readRouteFromFile(const QString &fileName, llh_t vehicleEnuRef);
bool isPointWithin(double px, double py, QList<PosPoint> route);
bool isPointWithin(PosPoint p, QList<PosPoint> route);

#endif // ROUTEUTILS_H