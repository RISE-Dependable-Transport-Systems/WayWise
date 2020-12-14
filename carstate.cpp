/*
    Copyright 2012 Benjamin Vedder	benjamin@vedder.se
              2020 Marvin Damschen  marvin.damschen@ri.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "carstate.h"
#include <QDebug>

CarState::CarState(int id, Qt::GlobalColor color)
{
    mId = id;
    mColor = color;
    mName = "";
    mName.sprintf("Car %d", mId);
    mTime = 0;
    mLength = 0.8;
    mWidth = 0.335;

    mPosition.setType(PosType::fused);
    mPositionGNSS.setType(PosType::GNSS);
    mPositionUWB.setType(PosType::UWB);
}

void CarState::draw(QPainter &painter, const QTransform &drawTrans, bool isSelected)
{
    PosPoint pos = getPosition();
//        LocPoint pos_gps = CarState->getLocationGps();

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
    //        LocPoint ap_goal = CarState->getApGoal();
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
    //        QTime t = QTime::fromMSecsSinceStartOfDay(CarState->getTime());
    //        QString solStr;
    //        if (!CarState->getLocationGps().getInfo().isEmpty()) {
    //            solStr = QString("Sol: %1\n").arg(CarState->getLocationGps().getInfo());
    //        }
    //        txt.sprintf("%s\n"
    //                    "%s"
    //                    "(%.3f, %.3f, %.0f)\n"
    //                    "%02d:%02d:%02d:%03d",
    //                    CarState->getName().toLocal8Bit().data(),
    //                    solStr.toLocal8Bit().data(),
    //                    pos.getX(), pos.getY(), angle,
    //                    t.hour(), t.minute(), t.second(), t.msec());
    //        pt_txt.setX(x + 120 + (car_len - 190) * ((cos(pos.getYaw()) + 1) / 2));
    //        pt_txt.setY(y);
    //        painter.setTransform(txtTrans);
    //        pt_txt = drawTrans.map(pt_txt);
    //        rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 20,
    //                           pt_txt.x() + 400, pt_txt.y() + 100);
    //        painter.drawText(rect_txt, txt);
}

int CarState::getId() const
{
    return mId;
}

void CarState::setId(int id, bool changeName)
{
    mId = id;

    if (changeName) {
        mName = "";
        mName.sprintf("Car %d", mId);
    }
}

QString CarState::getName() const
{
    return mName;
}

void CarState::setName(QString name)
{
    mName = name;
}

void CarState::setPosition(PosPoint &point)
{
    switch (point.getType()) {
        case PosType::fused:    mPosition = point; break;
        case PosType::GNSS:     mPositionGNSS = point; break;
        case PosType::UWB:      mPositionUWB = point; break;
    }
    emit carPositionUpdated(*this);
}

Qt::GlobalColor CarState::getColor() const
{
    return mColor;
}

void CarState::setColor(Qt::GlobalColor color)
{
    mColor = color;
}

PosPoint &CarState::getApGoal()
{
    return mApGoal;
}

void CarState::setApGoal(const PosPoint &apGoal)
{
    mApGoal = apGoal;
}

qint32 CarState::getTime() const
{
    return mTime;
}

void CarState::setTime(const qint32 &time)
{
    mTime = time;
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

double CarState::getSpeed() const
{
    return mSpeed;
}

void CarState::setSpeed(double value)
{
    mSpeed = value;
}

const QPointF CarState::getStoppingPointForTurnRadiusAndBrakeDistance(const double turnRadius, const double brakeDistance) const
{
    double x = fabs(turnRadius) * sin(brakeDistance/fabs(turnRadius));
    double y = turnRadius * (1 - cos(brakeDistance/fabs(turnRadius)));

    return QPointF(x, y);
}

const QPointF CarState::getStoppingPointForTurnRadius(const double turnRadius) const
{
    return getStoppingPointForTurnRadiusAndBrakeDistance(turnRadius, getBrakingDistance());
}

double CarState::getMinAcceleration() const
{
    return mMinAcceleration;
}

void CarState::setMinAcceleration(double minAcceleration)
{
    mMinAcceleration = minAcceleration;
}

double CarState::getMaxAcceleration() const
{
    return mMaxAcceleration;
}

void CarState::setMaxAcceleration(double maxAcceleration)
{
    mMaxAcceleration = maxAcceleration;
}

double CarState::getAxisDistance() const
{
    return mAxisDistance;
}

void CarState::setAxisDistance(double axisDistance)
{
    mAxisDistance = axisDistance;
}

double CarState::getLength() const
{
    return mLength;
}

void CarState::setLength(double length)
{
    mLength = length;
}

double CarState::getWidth() const
{
    return mWidth;
}

void CarState::setWidth(double width)
{
    mWidth = width;
}

PosPoint CarState::getPosition(PosType type) const
{
    switch (type) {
        case PosType::fused:    return mPosition;
        case PosType::GNSS:     return mPositionGNSS;
        case PosType::UWB:      return mPositionUWB;
    }

    return mPosition;
}
