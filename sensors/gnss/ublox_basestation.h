/*
 * Sets up u-blox GNSS reciver connected over serial as a basestation. Supports/tested with F9P/F9R only for now.
 */

#ifndef UBLOX_BASESTATION_H
#define UBLOX_BASESTATION_H

#include <QObject>
#include <QMap>
#include <QSerialPort>
#include <QSerialPortInfo>
#include "ublox.h"
#include "core/coordinatetransforms.h"

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
    bool isSerialConnected() {return mUblox.isSerialConnected();}
    BasestationConfig& getBasestationConfigCurrent();
    BasestationConfig& getBasestationConfigDefault();
    void pollMonVer();
    void pollCfgGNSS();

signals:
    void rtcmData(const QByteArray& data, const int& type);
    void currentPosition(const llh_t &llh);
    void rxNavSat(const ubx_nav_sat &sat);
    void rxSvin(const ubx_nav_svin &svin);
    void rxCfgGnss(const ubx_cfg_gnss &gnss);
    void rxMonVer(const QString &sw, const QString &hw, const QStringList &extensions);

private:
    Ublox mUblox;
    const int sendRtcmRefDelayMultiplier = 5;
    bool configureUblox(const BasestationConfig& basestationConfig);

};

#endif // UBLOX_BASESTATION_H
