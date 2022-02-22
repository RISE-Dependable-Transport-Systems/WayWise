#ifndef BNO055ORIENTATIONUPDATER_H
#define BNO055ORIENTATIONUPDATER_H

#include "imuorientationupdater.h"
extern "C" {
#include "ext/pi-bno055/getbno055.h"
}
#include <QTimer>
#include <QRunnable>

class ReadBNO055Task : public QObject, public QRunnable {
    Q_OBJECT
    void run() override
    {
        static struct bnoeul bnod;
        int res = get_eul(&bnod);

        emit gotBNOresult(qMakePair(res, bnod));
    }
signals:
    void gotBNOresult(const QPair<const int, const bnoeul> &result);
};

// connects to BNO055 via i2c bus and polls orientation data (euler angles) periodically
class BNO055OrientationUpdater : public IMUOrientationUpdater
{
public:
    BNO055OrientationUpdater(QSharedPointer<VehicleState> vehicleState, QString i2cBus = "/dev/i2c-1");
    void printBNO055Info();

    virtual bool setUpdateIntervall(int pollIntervall_ms) override;

private:
    const QString mBNO055I2CAddress = "0x28";
    int mPollIntervall_ms = 20;
    QTimer mPollTimer;
    ReadBNO055Task mReadBNO055Task;
};

#endif // BNO055ORIENTATIONUPDATER_H
