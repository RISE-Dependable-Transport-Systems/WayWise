/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 */

#include "trailerstate.h"


TrailerState::TrailerState(ObjectState::ObjectID_t id, Qt::GlobalColor color) : ObjectState (id, color)
{    

    mLength = 1;
    mWidth = 0.3;
    
}

#ifdef QT_GUI_LIB
void TrailerState::drawTrailer(QPainter &painter, const PosPoint &carPos, double car_len, double trailer_dist)
{
    double trailer_len = getLength() * 1000.0; // convert from m to mm (on the map objects are in mm)
    double x = carPos.getX() * 1000.0;
    double y = carPos.getY() * 1000.0;
    double trailer_x = x + car_len + trailer_dist;
    double trailer_y = y;
    
    // Draw trailer
    // simple draw a rectangle representing the trailer
    painter.setBrush(QBrush(Qt::white)); 
    painter.drawRect(trailer_x, trailer_y - trailer_len / 2, trailer_len, trailer_len);
}


void TrailerState::draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected){
    // draw a trailer
    PosPoint pos = getPosition();

    double trailer_len = getLength() * 1000.0; // convert from m to mm (on the map objects are in mm)
    double x = pos.getX() * 1000.0;
    double y = pos.getY() * 1000.0;
    double trailer_x = x + 100 + 10;
    double trailer_y = y;
    
    // Draw trailer
    // simple draw a rectangle representing the trailer
    painter.setBrush(QBrush(Qt::white)); 
    painter.drawRect(trailer_x, trailer_y - trailer_len / 2, trailer_len, trailer_len);


}
#endif