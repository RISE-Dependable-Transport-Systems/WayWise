/*
 *     Copyright 2024 Rickard Häll   rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Namespace to hold general geometric calculations
 */

#ifndef GEOMETRY_H
#define GEOMETRY_H


#include <QObject>
#include <QLineF>
#include <cmath>

namespace geometry {

inline QVector<QPointF> findIntersectionsBetweenCircleAndLine(QPair<QPointF,double> circle, QLineF line) {
    QVector<QPointF> intersections;

    double maxX = line.x1();
    double minX = line.x2();
    double maxY = line.y1();
    double minY = line.y2();
    if (maxX < minX) {
        maxX = line.x2();
        minX = line.x1();
    }
    if (maxY < minY) {
        maxY = line.y2();
        minY = line.y1();
    }

    double a = line.dx() * line.dx() + line.dy() * line.dy();
    double b = 2 * (line.dx() * (line.x1() - circle.first.x()) + line.dy() * (line.y1() - circle.first.y()));
    double c = (line.x1() - circle.first.x()) * (line.x1() - circle.first.x()) + (line.y1() - circle.first.y()) * (line.y1() - circle.first.y()) - circle.second * circle.second;

    double det = b * b - 4 * a * c;

    if ((a <= 1e-6) || (det < 0.0)) {
//         qDebug() << "No real solutions.";
    } else if (det == 0) {
//         qDebug() << "One solution.";
        double t = -b / (2 * a);
        double x = line.x1() + t * line.dx();
        double y = line.y1() + t * line.dy();

        if (x >= minX && x <= maxX &&
                y >= minY && y <= maxY)
            intersections.append(QPointF(x, y));
    } else {
//         qDebug() << "Two solutions.";
        double t = (-b + sqrtf(det)) / (2 * a);
        double x = line.x1() + t * line.dx();
        double y = line.y1() + t * line.dy();

        if (x >= minX && x <= maxX &&
                y >= minY && y <= maxY)
            intersections.append(QPointF(x, y));

        t = (-b - sqrtf(det)) / (2 * a);
        x = line.x1() + t * line.dx();
        y = line.y1() + t * line.dy();

        if (x >= minX && x <= maxX &&
                y >= minY && y <= maxY)
            intersections.append(QPointF(x, y));
    }

    return intersections;
}

}

#endif // GEOMETRY_H
