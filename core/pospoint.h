/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
    */

#ifndef POSPOINT_H
#define POSPOINT_H
#include <QObject>
#include <QPointF>
#include <QString>
#include <QTime>

enum class PosType {
    simulated,
    fused,
    odom,
    IMU,
    GNSS,
    UWB,
    _LAST_,
    defaultPosType = simulated
};
#include "coordinatetransforms.h"

class PosPoint : public QObject
{
    Q_OBJECT
public:

    PosPoint(double x = 0, double y = 0, double height = 0, double roll = 0,
             double pitch = 0, double yaw = 0, double speed = 0.5, double radius = 5.0,
             double sigma = 0.0, QTime time = QTime(),
             int id = 0, bool drawLine = true, quint32 attributes = 0, PosType type = PosType::simulated);
    PosPoint(const PosPoint &point);

    PosType getType() const;
    double getX() const;
    double getY() const;
    double getHeight() const;
    xyz_t getXYZ() const;
    double getRoll() const;
    double getPitch() const;
    double getYaw() const;
    double getSpeed() const;
    QPointF getPoint() const;
    QPointF getPointMm() const;
    double getRadius() const;
    double getSigma() const;
    QString getInfo() const;
    QColor getColor() const;
    QTime getTime() const;
    int getId() const;
    bool getDrawLine() const;
    quint32 getAttributes() const;
    double getDistanceTo(const PosPoint &point) const;
    double getDistanceTo3d(const PosPoint &point) const;

    void setType(const PosType &type);
    void setX(double x);
    void setY(double y);
    void setHeight(double height);
    void setXY(double x, double y);
    void setXYZ(xyz_t xyz);
    void scaleXY(double scalefactor);
    void setRoll(double roll);
    void setPitch(double pitch);
    void setYaw(double alpha);
    void setRollPitchYaw(double roll, double pitch, double yaw);
    void setSpeed(double speed);
    void setRadius(double radius);
    void setSigma(double sigma);
    void setInfo(const QString &info);
    void setColor(const QColor &color);
    void setTime(const QTime &time);
    void setId(int id);
    void setDrawLine(bool drawLine);
    void setAttributes(quint32 attributes);

    void updateWithOffsetAndYawRotation (xyz_t offset, double yaw);

    // Operators
    PosPoint& operator=(const PosPoint& point);
    bool operator==(const PosPoint& point);
    bool operator!=(const PosPoint& point);

private:
    double mX;
    double mY;
    double mHeight;
    double mRoll;
    double mPitch;
    double mYaw;
    double mSpeed;
    double mRadius;
    double mSigma;
    QString mInfo;
    QTime mTime;
    int mId;
    bool mDrawLine;
    quint32 mAttributes;
    PosType mType;

};

#endif // POSPOINT_H
