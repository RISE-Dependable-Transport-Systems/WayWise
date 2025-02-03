/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#ifndef ZIGZAGROUTEGENERATOR_H
#define ZIGZAGROUTEGENERATOR_H

#include <QPair>
#include "core/pospoint.h"


class ZigZagRouteGenerator
{
public:
    static bool isPointWithin(double px, double py, QList<PosPoint> route);
    static bool isPointWithin(PosPoint p, QList<PosPoint> route);
    static double distanceToLine(PosPoint p, PosPoint l0, PosPoint l1);
    static bool ccw(PosPoint a, PosPoint b, PosPoint c);
    static bool lineIntersect(PosPoint a, PosPoint b, PosPoint c, PosPoint d);
    static bool intersectionExists(QList<PosPoint> points0, QList<PosPoint> points1);
    static PosPoint getLineIntersection(QPair<PosPoint, PosPoint> line0, QPair<PosPoint, PosPoint> line1);
    static QList<PosPoint> getAllIntersections(QList<PosPoint> points0, QList<PosPoint> points1);
    static QList<PosPoint> getAllIntersections(QList<PosPoint> route);
    static int getClosestPointInRoute(PosPoint referencePoint, QList<PosPoint> route);
    static QPair<PosPoint,PosPoint> getBaselineDeterminingMinHeightOfConvexPolygon(QList<PosPoint> convexPolygon);
    static QList<PosPoint> fillConvexPolygonWithZigZag(QList<PosPoint> bounds, double spacing, bool keepTurnsInBounds, double speed, double speedInTurns, int turnIntermediateSteps, int visitEveryX,
                                                            uint32_t setAttributesOnStraights, uint32_t setAttributesInTurns, double attributeDistanceAfterTurn, double attributeDistanceBeforeTurn);
    static QList<PosPoint> fillConvexPolygonWithFramedZigZag(QList<PosPoint> bounds, double spacing, bool keepTurnsInBounds, double speed, double speedInTurns, int turnIntermediateSteps, int visitEveryX,
                                                                  uint32_t setAttributesOnStraights, uint32_t setAttributesInTurns, double attributeDistanceAfterTurn, double attributeDistanceBeforeTurn);
    static QList<PosPoint> getShrinkedConvexPolygon(QList<PosPoint> bounds, double spacing);
    static int getConvexPolygonOrientation(QList<PosPoint> bounds);

protected:
    ZigZagRouteGenerator();
};

#endif // ZIGZAGROUTEGENERATOR_H
