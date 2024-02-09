/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 */

#include "trailerstate.h"
#include <QDebug>

TrailerState::TrailerState(ObjectState::ObjectID_t id, Qt::GlobalColor color) : ObjectState (id, color)
{    

    mLength = 1.8;
    mWidth = 0.4;
    
}

#ifdef QT_GUI_LIB
void TrailerState::drawTrailer(QPainter &painter, const QTransform &drawTrans, const PosPoint &carPos, double angle)
{

    // qDebug() << "TrailerState::draw";
    // PosPoint pos = getPosition();

    //double trailer_len = getLength() * 500.0; // convert from m to mm (on the map objects are in mm)
    //qDebug() << "angle is " << angle;
    
    double trailerDistance= 100.0;
    double x = carPos.getX() * 1000.0  ;   // convert from m to mm (on the map objects are in mm)
    double y = carPos.getY() * 1000.0  ; 


   // Convert steering angle from degrees to radians
    double delta = 45 * M_PI / 180.0;

    // Calculate the angle of the trailer relative to the truck using the formula
    double trailerAngle = atan(10 / 4.5 * tan(delta));

    // Convert the angle from radians to degrees
    double sensorAngle = trailerAngle * (180.0 / M_PI);



    // x *= cos(sensorAngle);
    // y *= sin(-sensorAngle);
    double car_len = getLength() * 1000.0;
    const double car_w = getWidth() * 1000.0;
    const double car_corner = 0.02 * 1000.0;
    painter.setTransform(drawTrans);
    painter.translate(x, y);
    painter.rotate(carPos.getYaw() + sensorAngle + 180);

    // Wheels
    painter.setBrush(QBrush(Qt::darkGray));
    painter.drawRoundedRect(car_len - car_len / 2.5,-(car_w / 2), car_len / 9.0, car_w, car_corner / 3, car_corner / 3);
    
    // Draw trailer
    // simple draw a rectangle representing the trailer
    painter.setBrush(getColor());
    painter.drawRoundedRect(-car_len / 6.0, -((car_w - car_len / 20.0) / 2.0), car_len - (car_len / 20.0), car_w - car_len / 20.0, car_corner, car_corner);

    // QPen pen(Qt::black, 10);
    // painter.setPen(pen); // Set the pen color for drawing the axis
    // painter.drawLine(0, 0, -car_len / 12.0, 0);

}


void TrailerState::draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected){
    // draw a trailer
    qDebug() << "TrailerState::draw";

}
#endif