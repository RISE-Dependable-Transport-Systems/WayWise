/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 * 
 * Specific implementation of VehicleState for car-type (ackermann) vehicles, storing all (dynamic and static) state
 */

#include "truckstate.h"
#include <QDebug>
#include <QDateTime>

TruckState::TruckState(ObjectID_t id, Qt::GlobalColor color) : CarState(id, color)
{
    // Additional initialization if needed for the TruckState
}

void TruckState::updateOdomPositionAndYaw(double drivenDistance, PosType usePosType)
{
    // just Call the base class
    CarState::updateOdomPositionAndYaw(drivenDistance, usePosType);
}

#ifdef QT_GUI_LIB
void TruckState::draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected)
{
    PosPoint pos = getPosition();

    const double truck_len = getLength() * 1000.0;
    const double truck_w = getWidth() * 1000.0;
    const double truck_corner = 0.02 * 1000.0;

    double x = pos.getX() * 1000.0;
    double y = pos.getY() * 1000.0;
    painter.setTransform(drawTrans);

    QColor col_wheels;
    QColor col_bumper;
    QColor col_ap;
    QColor col_sigma = Qt::red;
    QColor col_hull = getColor();
    QColor col_center = Qt::blue;

    if (isSelected) {
        col_wheels = Qt::black;
        col_bumper = Qt::green;
        col_ap = getColor();
    } else {
        col_wheels = Qt::darkGray;
        col_bumper = Qt::lightGray;
        col_ap = Qt::lightGray;
    }

    // Draw standard deviation
    if (pos.getSigma() > 0.0) {
        QColor col = col_sigma;
        col.setAlphaF(0.2);
        painter.setBrush(QBrush(col));
        painter.drawEllipse(pos.getPointMm(), pos.getSigma() * 1000.0, pos.getSigma() * 1000.0);
    }

    // Draw truck
    painter.setBrush(QBrush(col_wheels));
    painter.save();
    painter.translate(x, y);
    painter.rotate(pos.getYaw());
    // Wheels
    painter.drawRoundedRect(-truck_len / 12.0,-(truck_w / 2), truck_len / 6.0, truck_w, truck_corner / 3, truck_corner / 3);
    painter.drawRoundedRect(truck_len - truck_len / 2.5,-(truck_w / 2), truck_len / 9.0, truck_w, truck_corner / 3, truck_corner / 3);
    // Front bumper
    painter.setBrush(col_bumper);
    painter.drawRoundedRect(-truck_len / 6.0, -((truck_w - truck_len / 20.0) / 2.0), truck_len, truck_w - truck_len / 20.0, truck_corner, truck_corner);
    // Hull
    painter.setBrush(col_hull);
    painter.drawRoundedRect(-truck_len / 6.0, -((truck_w - truck_len / 20.0) / 2.0), truck_len - (truck_len / 20.0), truck_w - truck_len / 20.0, truck_corner, truck_corner);
    
    // draw trailer if exist
    if (!mTrailerState.isNull()) {
        double angleInDegrees = getTrailerAngleDegrees();
        mTrailerState->drawTrailer(painter,drawTrans, pos, angleInDegrees);
    } else {
        // mTrailerState is empty
        qDebug() << "WARN: Trailer is empty";
    }

    painter.restore();
    
    painter.setBrush(col_center);
    painter.drawEllipse(QPointF(x, y), truck_w / 15.0, truck_w / 15.0);

    // Turning radius of truck
    painter.setPen(QPen(Qt::blue, 30));
    painter.setBrush(Qt::transparent);
    
    painter.drawEllipse(QPointF(x, y), getAutopilotRadius()*1000.0, getAutopilotRadius()*1000.0);
    painter.setPen(Qt::black);

    double trailerAngle  = getTrailerAngleRadians();
    double currYaw_rad = getPosition().getYaw() * (M_PI/180.0);
    double trailerYaw = currYaw_rad- trailerAngle;
    double trailerAxis = getTrailerWheelBase;
    double dx = trailerAxis * cos(trailerYaw);
    double dy = trailerAxis * sin(trailerYaw);
    double newX = (pos.getX() - dx) *1000.0;
    double newY = ( pos.getY() - dy) *1000.0;

    painter.setBrush(Qt::darkMagenta);
    painter.drawEllipse(QPointF(newX, newY), truck_w / 15.0, truck_w / 15.0);
     // Turning radius
    painter.setPen(QPen(Qt::darkMagenta, 20));
    painter.setBrush(Qt::transparent);
    painter.drawEllipse(QPointF(newX, newY), getAutopilotRadius()*1000.0, getAutopilotRadius()*1000.0);
    painter.setPen(Qt::black);


    QString txt;
    QPointF pt_txt;
    QRectF rect_txt;

    QTextStream txtStream(&txt);
    txtStream.setRealNumberPrecision(3);
    txtStream << "Trailer: " << Qt::endl
              << "(" << pos.getX()- dx << ", " << pos.getY()- dy << ", " 
              << trailerYaw / (M_PI/180.0)<< ")" << Qt::endl;
    pt_txt.setX(newX + truck_w + truck_len * ((cos(trailerYaw) - 1) / 3));
    pt_txt.setY(newY);
    painter.setTransform(txtTrans);
    pt_txt = drawTrans.map(pt_txt);
    rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 40,
                           pt_txt.x() + 400, pt_txt.y() + 65);
    painter.drawText(rect_txt, txt);

    if (getDrawStatusText()) {
        // Print data
        QString txt;
        QPointF pt_txt;
        QRectF rect_txt;

        QString flightModeStr;
        switch (getFlightMode()) {
            case FlightMode::Unknown: flightModeStr = "unknown"; break;
            case FlightMode::Ready: flightModeStr = "ready"; break;
            case FlightMode::Takeoff: flightModeStr = "takeoff"; break;
            case FlightMode::Hold: flightModeStr = "hold"; break;
            case FlightMode::Mission: flightModeStr = "mission"; break;
            case FlightMode::ReturnToLaunch: flightModeStr = "return to launch"; break;
            case FlightMode::Land: flightModeStr = "land"; break;
            case FlightMode::Offboard: flightModeStr = "offboard"; break;
            case FlightMode::FollowMe: flightModeStr = "follow me"; break;
            case FlightMode::Manual: flightModeStr = "manual"; break;
            case FlightMode::Altctl: flightModeStr = "altitude"; break;
            case FlightMode::Posctl: flightModeStr = "position"; break;
            case FlightMode::Acro: flightModeStr = "acro"; break;
            case FlightMode::Stabilized: flightModeStr = "stabilized"; break;
            case FlightMode::Rattitude: flightModeStr = "rattitude"; break;
        }

        QTextStream txtStream(&txt);
        txtStream.setRealNumberPrecision(3);
        txtStream << getName() << Qt::endl
                  << "(" << pos.getX() << ", " << pos.getY() << ", " << pos.getHeight() << ", " << (int)pos.getYaw() << ")" << Qt::endl
                  << "State: " << (getIsArmed() ? "armed" : "disarmed") << Qt::endl
                  << flightModeStr;

        pt_txt.setX(x + truck_w + truck_len * ((cos(getPosition().getYaw() * (M_PI/180.0)) + 1) / 3));
        pt_txt.setY(y);
        painter.setTransform(txtTrans);
        pt_txt = drawTrans.map(pt_txt);
        rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 40,
                           pt_txt.x() + 400, pt_txt.y() + 65);
        painter.drawText(rect_txt, txt);
    }
    
}

#endif