#ifndef IMUORIENTATIONUPDATER_H
#define IMUORIENTATIONUPDATER_H

#include <QObject>

class IMUOrientationUpdater : public QObject
{
    Q_OBJECT
public:
    explicit IMUOrientationUpdater(QObject *parent = nullptr);

signals:

};

#endif // IMUORIENTATIONUPDATER_H
