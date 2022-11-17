#include "zigzagroutegenerator.h"

ZigZagRouteGenerator::ZigZagRouteGenerator()
{

}

bool ZigZagRouteGenerator::isPointWithin(double px, double py, QList<PosPoint> route)
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

bool ZigZagRouteGenerator::isPointWithin(PosPoint p, QList<PosPoint> route)
{
    return isPointWithin(p.getX(),p.getY(),route);
}

double ZigZagRouteGenerator::distanceToLine(PosPoint p, PosPoint l0, PosPoint l1)
{
    double l = l0.getDistanceTo(l1);
    if (l < 0.01) {
        return p.getDistanceTo(l0);
    } // one cm

    double x = p.getX();
    double y = p.getY();
    double x1 = l0.getX();
    double y1 = l0.getY();
    double x2 = l1.getX();
    double y2 = l1.getY();
    double a = x - x1;
    double b = y - y1;
    double c = x2 - x1;
    double d = y2 - y1;

    double dot = a * c + b * d;
    double len_sq = c * c + d * d;

    double param = dot / len_sq;

    double xx, yy;

    if (param < 0) {
        xx = x1;
        yy = y1;
    }
    else if (param > 1) {
        xx = x2;
        yy = y2;
    }
    else {
        xx = x1 + param * c;
        yy = y1 + param * d;
    }

    double dx = x - xx;
    double dy = y - yy;
    return sqrt(dx * dx + dy * dy);
}

bool ZigZagRouteGenerator::ccw(PosPoint a, PosPoint b, PosPoint c)
{
    return (c.getY() - a.getY()) * (b.getX() - a.getX())
            > (b.getY() - a.getY()) * (c.getX() - a.getX());
}

bool ZigZagRouteGenerator::lineIntersect(PosPoint a, PosPoint b, PosPoint c, PosPoint d)
{
   return ccw(a,c,d) != ccw(b,c,d) && ccw(a,b,c) != ccw(a,b,d);
}

bool ZigZagRouteGenerator::intersectionExists(QList<PosPoint> points0, QList<PosPoint> points1)
{
    if (points0.size() < 2 || points1.size() < 2)
        return false;

    for (int i = 1; i < points0.size(); i++)
        for (int j = 1; j < points1.size(); j++)
            if (lineIntersect(points0.at(i-1), points0.at(i), points1.at(j-1), points1.at(j)))
                return true;

    return false;
}

PosPoint ZigZagRouteGenerator::getLineIntersection(QPair<PosPoint, PosPoint> line0, QPair<PosPoint, PosPoint> line1) // intersection can lie outside of segments!
{
    double x1=line0.first.getX(), y1=line0.first.getY(), x2=line0.second.getX(), y2=line0.second.getY(),
            x3=line1.first.getX(), y3=line1.first.getY(), x4=line1.second.getX(), y4=line1.second.getY();

    // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
    double xIntersect = ( ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4)) / ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)) );
    double yIntersect = ( ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4)) / ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)) );

    // qDebug() << "(" << x1 << "," << y1 << ")---(" << x2 << "," << y2 << ")  /  (" << x3 << "," << y3 << ")---(" << x4 << "," << y4 << ") => (" << intersection.getX() << "," << intersection.getY() << ")\n";

    return PosPoint(xIntersect, yIntersect);
}

QList<PosPoint> ZigZagRouteGenerator::getAllIntersections(QList<PosPoint> points0, QList<PosPoint> points1)
{
    QList<PosPoint> intersections;
    if (points0.size() < 2 || points1.size() < 2)
        return intersections;

    for (int i = 1; i < points0.size(); i++)
        for (int j = 1; j < points1.size(); j++)
            if (lineIntersect(points0.at(i-1), points0.at(i), points1.at(j-1), points1.at(j))) {
                intersections.append(getLineIntersection(QPair<PosPoint, PosPoint>(points0.at(i-1), points0.at(i)),
                                                         QPair<PosPoint, PosPoint>(points1.at(j-1), points1.at(j))));
            }

    return intersections;
}

QList<PosPoint> ZigZagRouteGenerator::getAllIntersections(QList<PosPoint> route)
{
    QList<PosPoint> intersections;
    if (route.size() < 2)
        return intersections;

    for (int i = 1; i < route.size(); i++)
        for (int j = i+2; j < route.size(); j++)
            if (lineIntersect(route.at(i-1), route.at(i), route.at(j-1), route.at(j)))
                intersections.append(getLineIntersection(QPair<PosPoint, PosPoint>(route.at(i-1), route.at(i)),
                                                         QPair<PosPoint, PosPoint>(route.at(j-1), route.at(j))));

    return intersections;
}

int ZigZagRouteGenerator::getClosestPointInRoute(PosPoint referencePoint, QList<PosPoint> route)
{
    double minDistance = std::numeric_limits<double>::max();
    int minIdx = -1;
    for (int i = 0; i < route.size(); i++) {
        double distance = sqrt(pow((route.at(i).getX()-referencePoint.getX()), 2) + pow((route.at(i).getY()-referencePoint.getY()), 2));
        if (distance < minDistance) {
            minDistance = distance;
            minIdx = i;
        }
    }
    assert(minIdx != -1);

    return minIdx;
}

QPair<PosPoint,PosPoint> ZigZagRouteGenerator::getBaselineDeterminingMinHeightOfConvexPolygon(QList<PosPoint> convexPolygon)
{
    // 1. Determine point with max distance for each line
    QList<QPair<QPair<PosPoint,PosPoint>, double>> maxDistances;
    for (int i = 0; i < convexPolygon.size() - 1 /* skip last */; i++) {
        PosPoint l0 = convexPolygon.at(i);
        PosPoint l1 = convexPolygon.at(i+1);

        double maxDistance = -1;
        for (int j = 0; j < convexPolygon.size(); j++) {
            if (j == i || j == i+1)
                continue;

            double currDistance = distanceToLine(convexPolygon.at(j), l0, l1);
            maxDistance = (currDistance > maxDistance)? currDistance : maxDistance;
        }
        maxDistances.append(QPair<QPair<PosPoint,PosPoint>, double>(QPair<PosPoint,PosPoint>(l0, l1), maxDistance));
    }
    assert(maxDistances.size() == convexPolygon.size() - 1);

    // 2. Determine line with minimum distance determined in 1.
    return std::min_element(maxDistances.begin(), maxDistances.end(),
                            [](QPair<QPair<PosPoint,PosPoint>, double> const& a, QPair<QPair<PosPoint,PosPoint>, double> const& b) -> bool
    {return a.second < b.second;})->first;
}

QList<PosPoint> ZigZagRouteGenerator::fillConvexPolygonWithZigZag(QList<PosPoint> bounds, double spacing, bool keepTurnsInBounds, double speed, double speedInTurns, int turnIntermediateSteps, int visitEveryX,
                                                        uint32_t setAttributesOnStraights, uint32_t setAttributesInTurns, double attributeDistanceAfterTurn, double attributeDistanceBeforeTurn)
{
    QList<PosPoint> route;

    // 1. resize bounds to ensure spacing (if applicable). TODO: not sure if helpful
//    if (distanceTowardsBounds)
//        bounds = getShrinkedConvexPolygon(bounds, spacing);

    // 2. get bound that determines optimal zigzag direction
    QPair<PosPoint, PosPoint> baseline = getBaselineDeterminingMinHeightOfConvexPolygon(bounds);
    double angle = atan2(baseline.second.getY() - baseline.first.getY(), baseline.second.getX() - baseline.first.getX());

    // 3. draw parallels to baseline that cover whole polygon
    int polygonDirectionSign = getConvexPolygonOrientation(bounds);

    PosPoint newLineBegin(baseline.first.getX() - 100000*cos(angle) + 0.01*polygonDirectionSign*cos(angle + M_PI/2),
                          baseline.first.getY() - 100000*sin(angle) + 0.01*polygonDirectionSign*sin(angle + M_PI/2));
    PosPoint newLineEnd(baseline.second.getX() + 100000*cos(angle) + 0.01*polygonDirectionSign*cos(angle + M_PI/2),
                        baseline.second.getY() + 100000*sin(angle) + 0.01*polygonDirectionSign*sin(angle + M_PI/2));

    while (intersectionExists(QList<PosPoint>({newLineBegin, newLineEnd}), bounds)) {
        route.append(newLineBegin);
        route.append(newLineEnd);

        newLineBegin = PosPoint(newLineBegin.getX() + polygonDirectionSign*spacing*cos(angle + M_PI/2), newLineBegin.getY() + polygonDirectionSign*spacing*sin(angle + M_PI/2));
        newLineEnd =   PosPoint(newLineEnd.getX() + polygonDirectionSign*spacing*cos(angle + M_PI/2), newLineEnd.getY() + polygonDirectionSign*spacing*sin(angle + M_PI/2));
    }

    // 4. cut parallels down to polygon ((i) ensure "short" connections of route are not within bounds, (ii) sort bounds to get reproducible results, (iii) get intersections with bounds)
    for (int i=1; i<route.size(); i+=4)
        std::swap(route[i-1], route[i]);

    int baselineStartInBounds = -1;
    for (int i = 0; i < bounds.size() && baselineStartInBounds == -1; i++)
        if (bounds.at(i).getX() == baseline.first.getX() && bounds.at(i).getY() == baseline.first.getY())
            baselineStartInBounds = i;
    assert(baselineStartInBounds > -1);

    QList<PosPoint> boundsStartingWithBaseline;
    for (int i = baselineStartInBounds; i < bounds.size(); i++)
        boundsStartingWithBaseline.append(bounds.at(i));
    for (int i = 0; i < baselineStartInBounds; i++)
        boundsStartingWithBaseline.append(bounds.at(i));
    bounds = boundsStartingWithBaseline;
    bounds.append(bounds.first());

    route = getAllIntersections(route, bounds);

    // 5. Adjust orientations, visit every xth line (if applies)
    for (int i = 1; i < route.size(); i+=4)
        std::swap(route[i-1], route[i]);

    QList<int> endsOfPasses;
    if (visitEveryX > 0) {
        QList<PosPoint> routeWIPeveryX;
        assert(visitEveryX % 2 == 0 && "not implemented for odd visitEveryX"); // TODO
        int currPoint = 0;
        for (int currPass = 1; currPass <= visitEveryX; currPass++) { // go back and forth to cover skipped lines
            // visit every everyX-th line
            for (; currPoint<route.size() && currPoint>=0; currPoint+=2*visitEveryX) {
                routeWIPeveryX.append(route.at(currPoint));
                (currPoint % 2) ? currPoint-- : currPoint++;
                routeWIPeveryX.append(route.at(currPoint));
            }
            currPoint -= 2*visitEveryX;
            // Go back to origin (skipM_PIng already-visited lines)
            if (currPass < visitEveryX)
                currPoint = currPass*2 + ((currPoint % 2) ? 0 : 1);

            endsOfPasses.append(routeWIPeveryX.size()-1);
        }
        route = routeWIPeveryX;
    }

    // 6. set speed and attributes, smoothen turns
    for (auto& pt : route)
        pt.setSpeed(speed);

    if (turnIntermediateSteps > 0) {
        const double turnRadius = (visitEveryX > 0) ? (visitEveryX*(spacing/2)) : (spacing/2);
        for (int i = 1; i < route.size()-1; i+=(2+turnIntermediateSteps)) {
            int turnDirectionSign = (i/(2+turnIntermediateSteps))%2 == 0 ? 1 : -1;

            int originalI = i - (i/(2+turnIntermediateSteps)*turnIntermediateSteps);
            bool turnToOrigin = false;
            if (endsOfPasses.contains(originalI))
                turnToOrigin = true; // go back to start a new pass

            PosPoint turnCenter;
            if (keepTurnsInBounds) {
                // Find turnCenter that guarantees to stay within bounds
                QPair<PosPoint, PosPoint> turnBound(PosPoint(route.at(i).getX(), route.at(i).getY()), PosPoint(route.at(i+1).getX(), route.at(i+1).getY()));
                double turnBoundAngle = atan2(turnBound.second.getY() - turnBound.first.getY(), turnBound.second.getX() - turnBound.first.getX());

                QPair<PosPoint, PosPoint> turnBoundShifted(PosPoint(route.at(i).getX() - turnRadius*turnDirectionSign*(-polygonDirectionSign)*cos(turnBoundAngle + M_PI/2),
                                                                    route.at(i).getY() - turnRadius*turnDirectionSign*(-polygonDirectionSign)*sin(turnBoundAngle + M_PI/2)),
                                                           PosPoint(route.at(i+1).getX() - turnRadius*turnDirectionSign*(-polygonDirectionSign)*cos(turnBoundAngle + M_PI/2),
                                                                    route.at(i+1).getY() - turnRadius*turnDirectionSign*(-polygonDirectionSign)*sin(turnBoundAngle + M_PI/2)));

                QPair<PosPoint, PosPoint> turnCenterLine(PosPoint(route.at(i).getX() + turnRadius*polygonDirectionSign*cos(angle + M_PI/2),
                                                                  route.at(i).getY() + turnRadius*polygonDirectionSign*sin(angle + M_PI/2)),
                                                         PosPoint(route.at(i-1).getX() + turnRadius*polygonDirectionSign*cos(angle + M_PI/2),
                                                                  route.at(i-1).getY() + turnRadius*polygonDirectionSign*sin(angle + M_PI/2)));

                turnCenter = getLineIntersection(turnBoundShifted, turnCenterLine);
            } else
                turnCenter = PosPoint(route.at(i).getX() + turnRadius*polygonDirectionSign*cos(angle + M_PI/2),
                                      route.at(i).getY() + turnRadius*polygonDirectionSign*sin(angle + M_PI/2));

            const double M_PIStep = M_PI/(turnIntermediateSteps+1) * turnDirectionSign * polygonDirectionSign;
            if (turnToOrigin) {
                // Turn going back to start a new pass. Draw two quarter circles with long connection inbetween
                PosPoint turnCenterA;
                PosPoint turnCenterB;
                if (keepTurnsInBounds) {
                    // Find turnCenters that guarantee to stay within bounds
                    QPair<PosPoint, PosPoint> turnBound(PosPoint(route.at(i+1).getX(), route.at(i+1).getY()), PosPoint(route.at(i).getX(), route.at(i).getY()));
                    double turnBoundAngle = atan2(turnBound.second.getY() - turnBound.first.getY(), turnBound.second.getX() - turnBound.first.getX());

                    QPair<PosPoint, PosPoint> turnBoundShifted(PosPoint(route.at(i).getX() - turnRadius*turnDirectionSign*(-polygonDirectionSign)*cos(turnBoundAngle + M_PI/2),
                                                                        route.at(i).getY() - turnRadius*turnDirectionSign*(-polygonDirectionSign)*sin(turnBoundAngle + M_PI/2)),
                                                               PosPoint(route.at(i+1).getX() - turnRadius*turnDirectionSign*(-polygonDirectionSign)*cos(turnBoundAngle + M_PI/2),
                                                                        route.at(i+1).getY() - turnRadius*turnDirectionSign*(-polygonDirectionSign)*sin(turnBoundAngle + M_PI/2)));

                    QPair<PosPoint, PosPoint> turnCenterLineA(PosPoint(route.at(i).getX() - turnRadius*polygonDirectionSign*cos(angle + M_PI/2),
                                                                      route.at(i).getY() - turnRadius*polygonDirectionSign*sin(angle + M_PI/2)),
                                                             PosPoint(route.at(i-1).getX() - turnRadius*polygonDirectionSign*cos(angle + M_PI/2),
                                                                      route.at(i-1).getY() - turnRadius*polygonDirectionSign*sin(angle + M_PI/2)));
                    QPair<PosPoint, PosPoint> turnCenterLineB(PosPoint(route.at(i+2).getX() + turnRadius*polygonDirectionSign*cos(angle + M_PI/2),
                                                                      route.at(i+2).getY() + turnRadius*polygonDirectionSign*sin(angle + M_PI/2)),
                                                             PosPoint(route.at(i+1).getX() + turnRadius*polygonDirectionSign*cos(angle + M_PI/2),
                                                                      route.at(i+1).getY() + turnRadius*polygonDirectionSign*sin(angle + M_PI/2)));

                    turnCenterA = getLineIntersection(turnBoundShifted, turnCenterLineA);
                    turnCenterB = getLineIntersection(turnBoundShifted, turnCenterLineB);
                } else {
                    turnCenterA = PosPoint(route.at(i).getX() - turnRadius*polygonDirectionSign*cos(angle + M_PI/2),
                                                    route.at(i).getY() - turnRadius*polygonDirectionSign*sin(angle + M_PI/2));
                    turnCenterB = PosPoint(route.at(i+1).getX() + turnRadius*polygonDirectionSign*cos(angle + M_PI/2),
                                                    route.at(i+1).getY() + turnRadius*polygonDirectionSign*sin(angle + M_PI/2));
                }

                for (int j = 0; j <= turnIntermediateSteps+1; j++) {
                    if (j <= (turnIntermediateSteps+1)/2)
                        turnCenter = turnCenterA;
                    else
                        turnCenter = turnCenterB;

                    PosPoint turnStep(turnCenter.getX() + turnRadius*cos(-j*M_PIStep + angle + M_PI/2 * polygonDirectionSign),
                                      turnCenter.getY() + turnRadius*sin(-j*M_PIStep + angle + M_PI/2 * polygonDirectionSign));

                    uint32_t tmpAttr = turnStep.getAttributes();
                    tmpAttr |= setAttributesInTurns;
                    turnStep.setAttributes(tmpAttr);

                    if (j == 0 || j == turnIntermediateSteps+1) {
                        if (keepTurnsInBounds) {
                            turnStep.setSpeed(speed);
                            route.replace(i+j,turnStep);
                        } else {
                            tmpAttr = route.at(i+j).getAttributes();
                            tmpAttr |= setAttributesInTurns;
                            route[i+j].setAttributes(tmpAttr);
                        }
                    } else {
                        turnStep.setSpeed(speedInTurns);
                        route.insert(i+j,turnStep);
                    }
                }
            } else
                // Standard turn. Draw half circle
                for (int j = 0; j <= turnIntermediateSteps+1; j++) {
                    PosPoint turnStep(turnCenter.getX() + turnRadius*cos(j*M_PIStep + angle - M_PI/2 * polygonDirectionSign),
                                      turnCenter.getY() + turnRadius*sin(j*M_PIStep + angle - M_PI/2 * polygonDirectionSign));

                    uint32_t tmpAttr = turnStep.getAttributes();
                    tmpAttr |= setAttributesInTurns;
                    turnStep.setAttributes(tmpAttr);

                    // Make sure that first and last point comply to bounds without breaking the turn
                    if ((j == 0 || j == turnIntermediateSteps+1)) {
                        if (keepTurnsInBounds || isPointWithin(turnStep, bounds)) {
                            turnStep.setSpeed(speed);
                            route.replace(i+j,turnStep);
                        } else {
                            tmpAttr = route.at(i+j).getAttributes();
                            tmpAttr |= setAttributesInTurns;
                            route[i+j].setAttributes(tmpAttr);
                        }
                    } else {
                        turnStep.setSpeed(speedInTurns);
                        route.insert(i+j,turnStep);
                    }
                }
        }
    }

    // 7. introduce points for predictable processing of attributes, set attributes
    if (setAttributesOnStraights) {
        for (int i = 1; i < route.size(); i+=(2+turnIntermediateSteps+2)) {
            int turnDirectionSign = (i/(2+turnIntermediateSteps+2))%2 == 0 ? 1 : -1;

            PosPoint attrStart(route.at(i-1).getX() + attributeDistanceAfterTurn*turnDirectionSign*cos(angle),
                               route.at(i-1).getY() + attributeDistanceAfterTurn*turnDirectionSign*sin(angle));
            PosPoint attrEnd(route.at(i).getX() - attributeDistanceBeforeTurn*turnDirectionSign*cos(angle),
                             route.at(i).getY() - attributeDistanceBeforeTurn*turnDirectionSign*sin(angle));

            uint32_t tmpAttr = attrStart.getAttributes();
            tmpAttr |= setAttributesOnStraights;
            attrStart.setAttributes(tmpAttr);

            tmpAttr = attrEnd.getAttributes();
            tmpAttr |= setAttributesOnStraights;
            attrEnd.setAttributes(tmpAttr);

            attrStart.setSpeed(speed);
            attrEnd.setSpeed(speed);

            route.insert(i,attrEnd);
            route.insert(i,attrStart);
        }
    }

    return route;
}

QList<PosPoint> ZigZagRouteGenerator::fillConvexPolygonWithFramedZigZag(QList<PosPoint> bounds, double spacing, bool keepTurnsInBounds, double speed, double speedInTurns, int turnIntermediateSteps, int visitEveryX,
                                                              uint32_t setAttributesOnStraights, uint32_t setAttributesInTurns, double attributeDistanceAfterTurn, double attributeDistanceBeforeTurn)
{
    QList<PosPoint> frame = getShrinkedConvexPolygon(bounds, spacing);

    QList<PosPoint> zigzag = fillConvexPolygonWithZigZag(frame, spacing, keepTurnsInBounds, speed, speedInTurns, turnIntermediateSteps, visitEveryX, setAttributesOnStraights, setAttributesInTurns, attributeDistanceAfterTurn, attributeDistanceBeforeTurn);

    int pointIdx = getClosestPointInRoute(zigzag.first(), frame);

    QList<PosPoint> route;
    for (int i = pointIdx; i < frame.size(); i++)
        route.append(frame.at(i));
    for (int i = 0; i < pointIdx; i++)
        route.append(frame.at(i));
    route.append(frame.at(pointIdx));

    for (auto& pt : route)
        pt.setSpeed(speed);
    route.append(zigzag);

    return route;
}

QList<PosPoint> ZigZagRouteGenerator::getShrinkedConvexPolygon(QList<PosPoint> bounds, double spacing)
{
    //bounds.append(bounds.at(0)); // NOTE: might make sense to make sure bounds are closed, but sometimes leads to strange results in this form
    int directionSign = getConvexPolygonOrientation(bounds);
    QList<PosPoint> shrinkedBounds;
    for (int i=1; i<bounds.size(); i++) {
        double angle = atan2(bounds.at(i).getY() - bounds.at(i-1).getY(), bounds.at(i).getX() - bounds.at(i-1).getX());
        PosPoint lineStart = PosPoint(bounds.at(i-1).getX() + directionSign*spacing*cos(angle + M_PI/2), bounds.at(i-1).getY() + directionSign*spacing*sin(angle + M_PI/2));
        PosPoint lineEnd = PosPoint(bounds.at(i).getX() + directionSign*spacing*cos(angle + M_PI/2), bounds.at(i).getY() + directionSign*spacing*sin(angle + M_PI/2));
        shrinkedBounds.append(lineStart);
        shrinkedBounds.append(lineEnd);
    }

    shrinkedBounds = getAllIntersections(shrinkedBounds);
    if (shrinkedBounds.size() > 1)
       std::swap(shrinkedBounds[0], shrinkedBounds[1]);
    shrinkedBounds.append(shrinkedBounds[0]);

    return shrinkedBounds;
}

int ZigZagRouteGenerator::getConvexPolygonOrientation(QList<PosPoint> bounds)
{
    if (bounds.size() < 3)
        return 0;

    return ((bounds[1].getX()*bounds[2].getY() + bounds[0].getX()*bounds[1].getY() + bounds[0].getY()*bounds[2].getX())
            -(bounds[0].getY()*bounds[1].getX() + bounds[1].getY()*bounds[2].getX() + bounds[0].getX()*bounds[2].getY())) > 0? 1 : -1;

}
