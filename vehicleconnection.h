#ifndef VEHICLECONNECTION_H
#define VEHICLECONNECTION_H

#include <QObject>
#include "core/coordinatetransforms.h"
#include "vehicles/vehiclestate.h"

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
