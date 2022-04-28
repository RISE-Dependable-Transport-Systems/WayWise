#ifndef RTCMCLIENT_H
#define RTCMCLIENT_H

#include <QObject>
#include <QTcpSocket>
#include <QHostAddress>
#include "core/coordinatetransforms.h"

#ifndef D
#define D(x) 						((double)x##L)
#endif

#ifndef D_PI
#define D_PI						D(3.14159265358979323846)
#endif

struct NtripConnectionInfo {
    QString user;
    QString password;
    QString stream;
};

class RtcmClient : public QObject
{
    Q_OBJECT
public:
    explicit RtcmClient(QObject *parent = nullptr);
    void connectTcp(QString host, qint16 port);
    void connectNtrip(QString host, qint16 port, NtripConnectionInfo ntripInfo);
    bool connectWithInfoFromFile(QString filePath);
    bool isConnected();
    void disconnect();
    static llh_t decodeLllhFromReferenceStationInfo(const QByteArray data);

    QString getCurrentHost() const;

    qint16 getCurrentPort() const;

signals:
    void rtcmData(const QByteArray &data);
    void baseStationPosition(const llh_t &baseStationPosition);

private:
    QTcpSocket mTcpSocket;
    QString mCurrentHost;
    qint16 mCurrentPort;
    NtripConnectionInfo mCurrentNtripConnectionInfo;

    // For parsing RTCMv3 (from RTKLIB)
    static const char RTCM3_PREAMBLE = 0xD3;
    static unsigned int getbitu(const char *buff, int pos, int len);
    static int getbits(const char *buff, int pos, int len);
    static double getbits_38(const char *buff, int pos);

};

#endif // RTCMCLIENT_H
