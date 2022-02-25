#ifndef VEHICLECONNECTION_H
#define VEHICLECONNECTION_H

#include <QObject>
#include "sdvp_qtcommon/coordinatetransforms.h"
#include "sdvp_qtcommon/vehiclestate.h"

class VehicleConnection : public QObject
{
    Q_OBJECT
public:
    explicit VehicleConnection(QObject *parent = nullptr);

    virtual void requestGotoENU(const xyz_t &xyz) = 0;

    QSharedPointer<VehicleState> getVehicleState() const;

signals:

protected:
    QSharedPointer<VehicleState> mVehicleState;

private:

};

#endif // VEHICLECONNECTION_H
