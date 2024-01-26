/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 * 
 * Specific implementation of VehicleState for car-type (ackermann) vehicles, storing all (dynamic and static) state
 */

#include "truckstate.h"
#include <QDebug>

TruckState::TruckState(int id, Qt::GlobalColor color) :
    CarState(id, color)
{
    // Additional initialization if needed for the TruckState
}

void TruckState::updateOdomPositionAndYaw(double drivenDistance, PosType usePosType)
{
    // Call the base class implementation first
    CarState::updateOdomPositionAndYaw(drivenDistance, usePosType);

    //double TrailerAngle = getTrailerAngle();
    //qDebug() << "Trailer angle " << TrailerAngle;

}

// Implement additional functions or overrides if needed
// For example, you might implement drawing functions if you added them in the header file.

#ifdef QT_GUI_LIB
// Additional implementation for drawing functions if needed
void TruckState::draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected)
{
    // Call the base class draw function to draw the truck
    CarState::draw(painter, drawTrans, txtTrans, isSelected);

    // add additional drawing logic for the trailer

    PosPoint pos = getPosition();

    const double truck_len = getLength() * 1000.0;
    const double truck_w = getWidth() * 1000.0;
    const double truck_corner = 0.02 * 1000.0;
    const double trailer_len = 1.5 * truck_len;  // Adjust the length of the trailer

    double x = pos.getX() * 1000.0;
    double y = pos.getY() * 1000.0;

    painter.setTransform(drawTrans);

    // Calculate trailer position
    double trailer_x = x - trailer_len * cos(pos.getYaw() * (M_PI / 180.0));
    double trailer_y = y - trailer_len * sin(pos.getYaw() * (M_PI / 180.0));

    // Draw trailer
    painter.save();
    painter.setBrush(QBrush(Qt::darkGray));  // Adjust the color of the trailer
    painter.translate(trailer_x, trailer_y);
    painter.rotate(pos.getYaw());
    painter.drawRoundedRect(-trailer_len / 2.0, -(truck_w / 2), trailer_len, truck_w, truck_corner / 3, truck_corner / 3);
    painter.restore();
}

// QPainterPath TruckState::getBoundingBox() const
// {
//     // Your bounding box logic here...
// }
#endif