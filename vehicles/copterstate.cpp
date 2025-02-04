/*
 *     Copyright 2020 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Specific implementation of VehicleState for multicopter drones/UAVs, storing all (dynamic and static) state
 */
#include "copterstate.h"
#include <QTime>
#include <QPalette>
#include <QTextStream>

CopterState::CopterState(ObjectID_t id, Qt::GlobalColor color) : VehicleState(id, color)
{
    // Default values for Holybro S500/X500
    mFrameType = CopterFrameType::X;
    mPropellerSize = 260;
    setWidth(520);
    setLength(520);

    ObjectState::setWaywiseObjectType(WAYWISE_OBJECT_TYPE_QUADCOPTER);
}

void CopterState::draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected)
{
//    LocPoint pos_gps = copterInfo.getLocationGps();
    PosPoint pos = getPosition();
    double x = pos.getX() * 1000.0;
    double y = pos.getY() * 1000.0;

//    double x_gps = pos_gps.getX() * 1000.0;
//    double y_gps = pos_gps.getY() * 1000.0;

    bool tooSmallForDetails = false;
    QLineF testScaleLine(0,0,0,1);
    double scale = drawTrans.map(testScaleLine).length();
    if (scale < 0.05)
        tooSmallForDetails = true;

    painter.setTransform(drawTrans);

    // Draw home
    QPen homePen;
    double scaleIndependentSize = 40/scale;
    homePen.setColor(Qt::red);
    homePen.setWidthF(scaleIndependentSize/10.0);
    painter.setPen(homePen);
    painter.setBrush(QBrush(Qt::lightGray));
    painter.drawEllipse(getHomePosition().getPoint()*1000.0, scaleIndependentSize-scaleIndependentSize/5.0, scaleIndependentSize-scaleIndependentSize/5.0);
    // "H"
    homePen.setWidthF(0.0);
    painter.setPen(homePen);
    painter.setBrush(QBrush(Qt::red));
    painter.drawRect(getHomePosition().getX()*1000.0-scaleIndependentSize/2.0,
                     getHomePosition().getY()*1000.0-scaleIndependentSize/2.0, scaleIndependentSize/5.0, scaleIndependentSize);
    painter.drawRect(getHomePosition().getX()*1000.0+scaleIndependentSize/2.0 - scaleIndependentSize/5.0,
                     getHomePosition().getY()*1000.0-scaleIndependentSize/2.0, scaleIndependentSize/5.0, scaleIndependentSize);
    painter.drawRect(getHomePosition().getX()*1000.0-scaleIndependentSize/2.0 + scaleIndependentSize/5.0,
                     getHomePosition().getY()*1000.0-scaleIndependentSize/10.0, scaleIndependentSize/5.0*3.0, scaleIndependentSize/5.0);

    // Draw copter
    QPen pen;
    pen.setColor(Qt::black);
    pen.setWidthF(10.0);
    painter.setPen(pen);
    painter.translate(x, y);
    painter.rotate(pos.getYaw() - 90.0);

    QColor col_frame = getColor();
    QColor col_prop_main;
    QColor col_prop_other;
//    QColor col_gps = Qt::magenta;
    QColor col_ap;

    if (isSelected) {
        col_prop_main = Qt::red;
        col_prop_other = Qt::green;
        col_ap = getColor();
    } else {
        col_prop_main = Qt::darkGray;
        col_prop_other = Qt::lightGray;
        col_ap = Qt::lightGray;
    }
    col_prop_main.setAlphaF(0.3);
    col_prop_other.setAlphaF(0.3);

    scaleIndependentSize = 30/scale;
    if (tooSmallForDetails) {
        pen.setColor(QPalette::WindowText);
        pen.setWidthF(2/scale);
        painter.setPen(pen);
        painter.setBrush(getColor());

        QPainterPath arrow;
        arrow.moveTo(0,-scaleIndependentSize/2);
        arrow.lineTo(-scaleIndependentSize, -scaleIndependentSize);
        arrow.lineTo(0, scaleIndependentSize);
        arrow.lineTo(scaleIndependentSize, -scaleIndependentSize);
        arrow.lineTo(0, -scaleIndependentSize/2);
        painter.drawPath(arrow);
    } else {
        painter.rotate((mFrameType == CopterFrameType::X) ? 45 : 0);

        // Draw the frame
        painter.setBrush(col_frame);
        painter.drawRect(-getWidth()/2, -20, getWidth(), 40);
        painter.drawRect(-20, -getLength()/2, 40, getLength());

        // Draw propellers
        painter.setBrush(QBrush(col_prop_main));
        painter.drawEllipse(QPointF(getWidth()/2, 0), mPropellerSize/2, mPropellerSize/2);
        if (mFrameType == CopterFrameType::PLUS)
            painter.setBrush(QBrush(col_prop_other));
        painter.drawEllipse(QPointF(0, getLength()/2), mPropellerSize/2, mPropellerSize/2);

        painter.setBrush(QBrush(col_prop_other));
        painter.drawEllipse(QPointF(0, -getLength()/2), mPropellerSize/2, mPropellerSize/2);
        painter.drawEllipse(QPointF(-getWidth()/2, 0), mPropellerSize/2, mPropellerSize/2);

        // Draw velocity
        if (fabs(getVelocity().x) > 1e-5 || fabs(getVelocity().y) > 1e-5) {
            painter.rotate((mFrameType == CopterFrameType::X) ? -45+pos.getYaw() : pos.getYaw());
            painter.setBrush(QBrush(Qt::green));
            painter.setPen(QPen(Qt::green, 30));
            painter.drawLine(QPointF(0.0, 0.0), QPointF(getVelocity().x*1000.0, getVelocity().y*1000.0));
        }
    }

    // Print data
    QString txt;
    QPointF pt_txt;
    QRectF rect_txt;

    QString landedStateStr;
    switch (mLandedState) {
        case LandedState::Unknown: landedStateStr = "unknown"; break;
        case LandedState::OnGround: landedStateStr = "on ground"; break;
        case LandedState::InAir: landedStateStr = "in air"; break;
        case LandedState::TakingOff: landedStateStr = "taking off"; break;
        case LandedState::Landing: landedStateStr = "landing"; break;
    }
    
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
              << "State: " << (getIsArmed() ? "armed, " : "disarmed, ") << landedStateStr << Qt::endl
              << flightModeStr;

    pt_txt.setX(x + ((scale < 0.05) ? scaleIndependentSize : (getWidth() + getLength())/2));
    pt_txt.setY(y);
    painter.setTransform(txtTrans);
    pt_txt = drawTrans.map(pt_txt);
    rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 40,
                       pt_txt.x() + 400, pt_txt.y() + 65);
    painter.setPen(QPen(QPalette::WindowText));
    painter.drawText(rect_txt, txt);

    // Restore transform
    painter.setTransform(drawTrans);

//    // Autopilot state
//    LocPoint ap_goal = getApGoal();
//    if (ap_goal.getRadius() > 0.0) {
//        QPointF p = ap_goal.getPointMm();
//        pen.setColor(col_ap);
//        painter.setPen(pen);
//        painter.drawEllipse(p, 10 / mScaleFactor, 10 / mScaleFactor);
//    }

//    // GPS Location
//    painter.setBrush(col_gps);
//    painter.drawEllipse(QPointF(x_gps, y_gps), 335.0 / 15.0, 335.0 / 15.0);
}

void CopterState::updateOdomPositionAndYaw(double drivenDistance, PosType usePosType)
{
    PosPoint currentPosition = getPosition(usePosType);

    double currVelocityMagnitude = sqrt(getVelocity().x*getVelocity().x + getVelocity().y*getVelocity().y + getVelocity().z*getVelocity().z);
    Velocity currVelocityNormalized = {currVelocityMagnitude*getVelocity().x, currVelocityMagnitude*getVelocity().y, currVelocityMagnitude*getVelocity().z};

    currentPosition.setX(currentPosition.getX() + drivenDistance*currVelocityNormalized.x);
    currentPosition.setY(currentPosition.getY() + drivenDistance*currVelocityNormalized.y);
    currentPosition.setHeight(currentPosition.getHeight() + drivenDistance*currVelocityNormalized.z);

    currentPosition.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
    setPosition(currentPosition);
}

double CopterState::steeringCurvatureToSteering(double steeringCurvature)
{
    // behaves similar to a diffdrive vehicle in the xy-plane
    return (getWidth() / 2.0) * steeringCurvature;
}

CopterState::LandedState CopterState::getLandedState() const
{
    return mLandedState;
}

void CopterState::setLandedState(const CopterState::LandedState &landedState)
{
    mLandedState = landedState;
}
