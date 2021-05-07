#include "carstate.h"
#include <QDebug>

CarState::CarState(int id, Qt::GlobalColor color) : VehicleState(id, color)
{

}

#ifdef QT_GUI_LIB
void CarState::draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected)
{
    PosPoint pos = getPosition();
//        LocPoint pos_gps = VehicleState->getLocationGps();

    const double car_len = getLength() * 1000.0;
    const double car_w = getWidth() * 1000.0;
    const double car_corner = 0.02 * 1000.0;

    double x = pos.getX() * 1000.0;
    double y = pos.getY() * 1000.0;
//        double x_gps = pos_gps.getX() * 1000.0;
//        double y_gps = pos_gps.getY() * 1000.0;
    double angle = pos.getYaw() * 180.0 / M_PI;
    painter.setTransform(drawTrans);

    QColor col_wheels;
    QColor col_bumper;
    QColor col_ap;
    QColor col_sigma = Qt::red;
    QColor col_hull = getColor();
    QColor col_center = Qt::blue;
//        QColor col_gps = Qt::magenta;

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

    // Draw car
    painter.setBrush(QBrush(col_wheels));
    painter.save();
    painter.translate(x, y);
    painter.rotate(-angle);
    // Wheels
    painter.drawRoundedRect(-car_len / 12.0,-(car_w / 2), car_len / 6.0, car_w, car_corner / 3, car_corner / 3);
    painter.drawRoundedRect(car_len - car_len / 2.5,-(car_w / 2), car_len / 6.0, car_w, car_corner / 3, car_corner / 3);
    // Front bumper
    painter.setBrush(col_bumper);
    painter.drawRoundedRect(-car_len / 6.0, -((car_w - car_len / 20.0) / 2.0), car_len, car_w - car_len / 20.0, car_corner, car_corner);
    // Hull
    painter.setBrush(col_hull);
    painter.drawRoundedRect(-car_len / 6.0, -((car_w - car_len / 20.0) / 2.0), car_len - (car_len / 20.0), car_w - car_len / 20.0, car_corner, car_corner);
    painter.restore();

    // Center
    painter.setBrush(col_center);
    painter.drawEllipse(QPointF(x, y), car_w / 15.0, car_w / 15.0);

    //        // GPS Location
    //        painter.setBrush(col_gps);
    //        painter.drawEllipse(QPointF(x_gps, y_gps), car_w / 15.0, car_w / 15.0);

    //        // Autopilot state
    //        LocPoint ap_goal = VehicleState->getApGoal();
    //        if (ap_goal.getRadius() > 0.0) {
    //            pen.setColor(col_ap);
    //            painter.setPen(pen);
    //            QPointF pm = pos.getPointMm();
    //            painter.setBrush(Qt::transparent);
    //            painter.drawEllipse(pm, ap_goal.getRadius() * 1000.0, ap_goal.getRadius() * 1000.0);

    //            QPointF p = ap_goal.getPointMm();
    //            pen.setWidthF(3.0 / mScaleFactor);
    //            painter.setBrush(col_gps);
    //            painter.drawEllipse(p, 10 / mScaleFactor, 10 / mScaleFactor);
    //        }

    //        painter.setPen(QPen(textColor));

    //        // Print data
    //        QTime t = QTime::fromMSecsSinceStartOfDay(VehicleState->getTime());
    //        QString solStr;
    //        if (!VehicleState->getLocationGps().getInfo().isEmpty()) {
    //            solStr = QString("Sol: %1\n").arg(VehicleState->getLocationGps().getInfo());
    //        }
    //        txt.sprintf("%s\n"
    //                    "%s"
    //                    "(%.3f, %.3f, %.0f)\n"
    //                    "%02d:%02d:%02d:%03d",
    //                    VehicleState->getName().toLocal8Bit().data(),
    //                    solStr.toLocal8Bit().data(),
    //                    pos.getX(), pos.getY(), angle,
    //                    t.hour(), t.minute(), t.second(), t.msec());

    if (getDrawStatusText()) {
        QPointF statusTextPoint;
        QRectF statusTextRect;
        QString statusText = QString::number(getId());

        statusTextPoint.setX(x + car_w + car_len * ((cos(getPosition().getYaw()) + 1) / 3));
        statusTextPoint.setY(y - car_w / 2);

        painter.setTransform(txtTrans);
        statusTextPoint = drawTrans.map(statusTextPoint);
        statusTextRect.setCoords(statusTextPoint.x(), statusTextPoint.y(),
                           statusTextPoint.x() + 400, statusTextPoint.y() + 100);
        painter.drawText(statusTextRect, statusText);
    }
}
#endif

void CarState::simulationStep(double dt_ms)
{
    PosPoint currentPosition = getPosition();
    double drivenDistance = getSpeed() * dt_ms / 1000;

    // Bicycle kinematic model with rear axle as reference point
    if (fabs(getSteering()) > 1e-6) { // Turning
        double turnRadiusRear = getTurnRadiusRear();
        double turnRadiusFront = getTurnRadiusFront();

        if (turnRadiusRear < 0.0) {
            turnRadiusFront = -turnRadiusFront;
        }

        double yawChange = drivenDistance / ((turnRadiusRear + turnRadiusFront) / 2.0);

        currentPosition.setX(currentPosition.getX() + turnRadiusRear * (sin(-currentPosition.getYaw() + yawChange) - sinf(-currentPosition.getYaw())));
        currentPosition.setY(currentPosition.getY() + turnRadiusRear * (cos(-currentPosition.getYaw() - yawChange) - cosf(-currentPosition.getYaw())));

        currentPosition.setYaw(currentPosition.getYaw() - yawChange);
    } else { // Driving forward
        currentPosition.setX(currentPosition.getX() + cos(-currentPosition.getYaw()) * drivenDistance);
        currentPosition.setY(currentPosition.getY() + sin(-currentPosition.getYaw()) * drivenDistance);
    }

    setPosition(currentPosition);
}

double CarState::getAxisDistance() const
{
	return fabs(mAxisDistance) < 0.001 ? 0.8*getLength() : mAxisDistance;
}

void CarState::setAxisDistance(double axisDistance)
{
    mAxisDistance = axisDistance;
}

double CarState::getSteering() const
{
    return mSteering;
}

void CarState::setSteering(double value)
{
    value = (value > tanf(getMaxSteeringAngle())) ? tanf(getMaxSteeringAngle()) : value;
    value = (value < -tanf(getMaxSteeringAngle())) ? -tanf(getMaxSteeringAngle()) : value;
    mSteering = value;
}

void CarState::setMaxSteeringAngle(double steeringAngle_rad) {
	mMaxSteeringAngle = fabs(steeringAngle_rad);
}

void CarState::setMinTurnRadiusRear(double minTurnRadius_m) {
	mMinTurnRadiusRear = fabs(minTurnRadius_m);
}

double CarState::getBrakingDistance() const {
    double brakingDistance = 0.0;
    if (getSpeed() != 0.0)
        brakingDistance = getSpeed() * getTotalReactionTime() -
                ((getSpeed() >= 0.0) ? 0.5 : -0.5) * pow(abs(getSpeed()) + getMaxAcceleration() * getTotalReactionTime(), 2.0) / getMinAcceleration();

    return brakingDistance;
}

double CarState::getThreeSecondsDistance() const
{
    return 3.0 * getSpeed();
}

const QPointF CarState::getStoppingPointForTurnRadiusAndBrakingDistance(const double turnRadius, const double brakingDistance) const
{
    double x = fabs(turnRadius) * sin(brakingDistance/fabs(turnRadius));
    double y = turnRadius * (1 - cos(brakingDistance/fabs(turnRadius)));

    return QPointF(x, y);
}

const QPointF CarState::getStoppingPointForTurnRadius(const double turnRadius) const
{
    return getStoppingPointForTurnRadiusAndBrakingDistance(turnRadius, getBrakingDistance());
}
