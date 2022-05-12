#ifndef GIMBAL_H
#define GIMBAL_H

#include <QObject>
#include "core/coordinatetransforms.h"

class Gimbal : public QObject
{
    Q_OBJECT
public:
    virtual void setRegionOfInterest(const llh_t& roiLlh) = 0;
    void setRegionOfInterest(const xyz_t& roiENU, const llh_t& enuReference) {
        setRegionOfInterest(coordinateTransforms::enuToLlh(enuReference, roiENU));
    }
    virtual void setPitchAndYaw(double pitch_deg, double yaw_deg) = 0;


signals:

};

#endif // GIMBAL_H
