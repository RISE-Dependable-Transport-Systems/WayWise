#include "copterstate.h"
#include <QTime>
#include <QPalette>

CopterState::CopterState(int id, Qt::GlobalColor color)
{

}

void CopterState::draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected)
{
//    pen.setWidthF(2.0 / mScaleFactor);
//    pen.setColor(textColor);
//    painter.setPen(pen);

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

    QColor col_frame = getColor();
    QColor col_prop_main;
    QColor col_prop_other;
    QColor col_gps = Qt::magenta;
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

    // Draw the Quadcopter
    painter.setBrush(col_frame);
    painter.drawRoundedRect(-20, -300, 40, 600, 10, 10);
    painter.drawRoundedRect(-300, -20, 600, 40, 10, 10);

    // Draw propellers
    QColor col = col_prop_main;
    col.setAlphaF(0.3);
    painter.setBrush(QBrush(col));
    painter.drawEllipse(QPointF(275, 0), 130, 130);
    col = col_prop_other;
    col.setAlphaF(0.3);
    painter.setBrush(QBrush(col));
    painter.drawEllipse(QPointF(0, 275), 130, 130);
    painter.drawEllipse(QPointF(0, -275), 130, 130);
    painter.drawEllipse(QPointF(-275, 0), 130, 130);

    // Draw the acceleration vector
    if (fabs(pos.getRoll()) > 1e-5 || fabs(pos.getPitch()) > 1e-5) {
        painter.setBrush(QBrush(Qt::green));
        painter.setPen(QPen(Qt::green, 30));
        painter.drawLine(QPointF(0.0, 0.0), QPointF(-pos.getPitch() * 800.0, -pos.getRoll() * 800.0));
    }

    // Print data
    QTime t = QTime::fromMSecsSinceStartOfDay(getTime());
    QString txt;
    QPointF pt_txt;
    QRectF rect_txt;
    txt.sprintf("%s\n"
                "(%.3f, %.3f, %.0f)\n"
                "%02d:%02d:%02d:%03d",
                getName().toLocal8Bit().data(),
                pos.getX(), pos.getY(), angle,
                t.hour(), t.minute(), t.second(), t.msec());
    pt_txt.setX(x + 450);
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
