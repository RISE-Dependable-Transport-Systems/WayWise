#include "copterstate.h"
#include <QTime>
#include <QPalette>

CopterState::CopterState(int id, Qt::GlobalColor color) : VehicleState(id, color)
{
    // Default values for Holybro S500/X500
    mFrameType = CopterFrameType::X;
    mPropellerSize = 260;
    setWidth(520);
    setLength(520);
}

void CopterState::draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected)
{
//    LocPoint pos_gps = copterInfo.getLocationGps();
    PosPoint pos = getPosition();
    double x = pos.getX() * 1000.0;
    double y = pos.getY() * 1000.0;

//    double x_gps = pos_gps.getX() * 1000.0;
//    double y_gps = pos_gps.getY() * 1000.0;
    double angle = pos.getYaw() * 180.0 / M_PI;

    painter.setTransform(drawTrans);
    painter.translate(x, y);
    painter.rotate(-angle);

    bool tooSmallForDetails = false;
    QLineF testScaleLine(0,0,0,1);
    double scale = drawTrans.map(testScaleLine).length();
    if (scale < 0.05)
        tooSmallForDetails = true;

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

    double scaleIndependentSize = 30/scale;
    if (tooSmallForDetails) {
        QPen pen;
        pen.setColor(QPalette::Foreground);
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
            painter.rotate((mFrameType == CopterFrameType::X) ? -45+angle : angle);
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


    txt.sprintf("%s\n"
                "(%.3f, %.3f, %.3f, %.0f)\n"
                "State: %s, %s\n",
                getName().toLocal8Bit().data(),
                pos.getX(), pos.getY(), pos.getHeight(), angle,
                (getIsArmed() ? "armed" : "disarmed"),
                landedStateStr.toLocal8Bit().data());
    pt_txt.setX(x + ((scale < 0.05) ? scaleIndependentSize : (getWidth() + getLength())/2));
    pt_txt.setY(y);
    painter.setTransform(txtTrans);
    pt_txt = drawTrans.map(pt_txt);
    rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 40,
                       pt_txt.x() + 400, pt_txt.y() + 45);
    painter.setPen(QPen(QPalette::Foreground));
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
