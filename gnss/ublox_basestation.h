#ifndef UBLOX_BASESTATION_H
#define UBLOX_BASESTATION_H

#include <QObject>
#include <QMap>
#include <QSerialPort>
#include <QSerialPortInfo>
#include "ublox.h"

// Sets up Ublox GNSS connected over serial as a basestation. Supports/tested with F9P only for now.
// Does not use/provide any widget classes to enable use in cmd applications.
class UbloxBasestation : public QObject
{
    Q_OBJECT
public:
    enum class BasestationMode {Fixed, MovingBase, SurveyIn};
    struct BasestationConfig {
        BasestationMode mode = BasestationMode::SurveyIn;
        unsigned baudrate = 921600;
        unsigned measurementRate = 1; // [Hz]
        unsigned navSolutionRate = 1; // measurements per solution
        double fixedRefLat = -1; // position to send in 'Fixed' mode
        double fixedRefLon = -1;
        double fixedRefHeight = -1;
        double surveyInMinAcc = 2.0; // [m]
        unsigned surveyInMinDuration = 60; // [s]
    };
    static const BasestationConfig defaultConfig;

    explicit UbloxBasestation(QObject *parent = nullptr);
    bool connectSerial(const QSerialPortInfo &serialPortInfo, const BasestationConfig config = defaultConfig);
    bool disconnectSerial();
    BasestationConfig& getBasestationConfigCurrent();
    BasestationConfig& getBasestationConfigDefault();

signals:
    void rtcmData(const QByteArray& data, const int& type);
    void currentPosition(const double& refLat, const double& refLon, const double& refHeight);

private:
    Ublox mUblox;
    QMap<int, int> mRtcmUbx;
    const int sendRtcmRefDelayMultiplier = 5;
    bool configureUblox(const BasestationConfig& basestationConfig);

private slots:
    void rxNavPvt(ubx_nav_pvt pvt);
    void rxNavSat(ubx_nav_sat sat);
    void rxSvin(ubx_nav_svin svin);
    void rtcmRx(const QByteArray &data, const int& type);

};

#endif // UBLOX_BASESTATION_H
