#ifndef UBLOXROVER_H
#define UBLOXROVER_H

#include <QObject>
#include "ublox.h"

class UbloxRover : public QObject
{
    Q_OBJECT
public:
    explicit UbloxRover(QObject *parent = nullptr);
    bool connectSerial(const QSerialPortInfo &serialPortInfo);

signals:

private:
    bool configureUblox();

    Ublox mUblox;

};

#endif // UBLOXROVER_H
