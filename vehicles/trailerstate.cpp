/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 */

#include "trailerstate.h"
#include <QDebug>

TrailerState::TrailerState(ObjectState::ObjectID_t id, Qt::GlobalColor color) : ObjectState (id, color)
{    

    mLength = 1.0;
    mWidth = 0.185;
    
}

#ifdef QT_GUI_LIB
void TrailerState::drawTrailer(QPainter &painter, const QTransform &drawTrans, const PosPoint &carPos, double angle)
{

    double x = carPos.getX() * 1000.0  ;// convert from m to mm (on the map objects are in mm)
    double y = carPos.getY() * 1000.0  ;
    double trailer_len = getLength() * 1000.0;
    const double trailer_w = getWidth() * 1000.0;
    const double trailer_corner = 0.02 * 1000.0;

    painter.setTransform(drawTrans);
    painter.translate(x, y);
    painter.rotate(carPos.getYaw() - angle + 180);

    // Wheels
    painter.setBrush(QBrush(Qt::darkGray));
    painter.drawRoundedRect(trailer_len - trailer_len / 2.5,-(trailer_w / 2), trailer_len / 9.0, trailer_w, trailer_corner / 3, trailer_corner / 3);
    
    // Draw trailer
    // simple draw a rectangle representing the trailer
    painter.setBrush(getColor());
    painter.drawRoundedRect(-trailer_len / 6.0, -((trailer_w - trailer_len / 20.0) / 2.0), trailer_len - (trailer_len / 20.0), trailer_w - trailer_len / 20.0, trailer_corner, trailer_corner);

    // QPen pen(Qt::black, 10);
    // painter.setPen(pen); // Set the pen color for drawing the axis
    // painter.drawLine(0, 0, -trailer_len / 12.0, 0);

}


void TrailerState::draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected){
    // draw a trailer indepentently 
     
    PosPoint pos = getPosition();
    double x = pos.getX() * 1000.0  ;// convert from m to mm (on the map objects are in mm)
    double y = pos.getY() * 1000.0  ;
    double trailer_len = getLength() * 1000.0;
    const double trailer_w = getWidth() * 1000.0;
    const double trailer_corner = 0.02 * 1000.0;

    painter.setTransform(drawTrans);
    painter.translate(x, y);
    painter.rotate(pos.getYaw() - this->mAngle + 180);

    // Wheels
    painter.setBrush(QBrush(Qt::darkGray));
    painter.drawRoundedRect(trailer_len - trailer_len / 2.5,-(trailer_w / 2), trailer_len / 9.0, trailer_w, trailer_corner / 3, trailer_corner / 3);
    
    // Draw trailer
    // simple draw a rectangle representing the trailer
    painter.setBrush(getColor());
    painter.drawRoundedRect(-trailer_len / 6.0, -((trailer_w - trailer_len / 20.0) / 2.0), trailer_len - (trailer_len / 20.0), trailer_w - trailer_len / 20.0, trailer_corner, trailer_corner);

}
#endif