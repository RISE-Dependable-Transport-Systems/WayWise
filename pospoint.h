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

#ifndef POSPOINT_H
#define POSPOINT_H
#include <QObject>
#include <QPointF>
#include <QString>

enum class PosType {
    simulated,
    fused,
    odom,
    IMU,
    GNSS,
    UWB,
    _LAST_
};

class PosPoint : public QObject
{
    Q_OBJECT
public:

	PosPoint(double x = 0, double y = 0, double height = 0, double roll = 0,
			 double pitch = 0, double yaw = 0, double speed = 0.5, double radius = 5.0,
			 double sigma = 0.0, qint32 time = 0,
			 int id = 0, bool drawLine = true, quint32 attributes = 0, PosType type = PosType::simulated);
    PosPoint(const PosPoint &point);

    PosType getType() const;
    double getX() const;
    double getY() const;
    double getHeight() const;
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
    qint32 getTime() const;
    int getId() const;
    bool getDrawLine() const;
    quint32 getAttributes() const;
    double getDistanceTo(const PosPoint &point) const;
    double getDistanceTo3d(const PosPoint &point) const;
	double getVarianceX() const;
	double getVarianceY() const;
	double getCovarianceXY() const;

    void setType(const PosType &type);
    void setX(double x);
    void setY(double y);
    void setHeight(double height);
    void setXY(double x, double y);
    void scaleXY(double scalefactor);
    void setRoll(double roll);
    void setPitch(double pitch);
    void setYaw(double alpha);
    void setSpeed(double speed);
    void setRadius(double radius);
    void setSigma(double sigma);
    void setInfo(const QString &info);
    void setColor(const QColor &color);
    void setTime(const qint32 &time);
    void setId(int id);
    void setDrawLine(bool drawLine);
    void setAttributes(quint32 attributes);
	void setVarianceX(double varX);
	void setVarianceY(double varY);
	void setCovarianceXY(double covXY);

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
    qint32 mTime;
    int mId;
    bool mDrawLine;
    quint32 mAttributes;
    PosType mType;
	double mVarianceX = 0.0;
	double mVarianceY = 0.0;
	double mCovarianceXY = 0.0;

};

#endif // POSPOINT_H
