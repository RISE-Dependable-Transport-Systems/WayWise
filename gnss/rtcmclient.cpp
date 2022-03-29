#include "rtcmclient.h"
#include <QFile>

RtcmClient::RtcmClient(QObject *parent) : QObject(parent)
{
    connect(&mTcpSocket, &QTcpSocket::readyRead, [this]{
        QByteArray data =  mTcpSocket.readAll();

        // Make sure to skip "ICY 200 OK" when connection to NTRIP
        static bool skipFirstReply = true;
        if (skipFirstReply) {
            skipFirstReply = false;
            return;
        }

        // Try to read 1005 or 1006 from stream to get base station position
        // See RTKLIB for how RTCM is decoded (https://github.com/tomojitakasu/RTKLIB)
        // We are not CRC checking here (getting data via TCP anyways)
        // TODO: wait for rest of incomplete message?
        static bool foundReferenceStationInfo = false;
        if (!foundReferenceStationInfo) {
            const char* dataPtr = data.constData();
            while ((dataPtr = std::find(dataPtr, data.constEnd(), RTCM3_PREAMBLE)) != data.constEnd()) {
                if (dataPtr + 5 < data.constEnd()) {
                    int length = getbitu(dataPtr, 14, 10) + 1; // number of bytes inkl. crc
                    int type = getbitu(dataPtr, 24, 12);
                    if (dataPtr + length < data.constEnd() && (type == 1005 || type == 1006)) {
                        llh_t baseLlh = decodeLllhFromReferenceStationInfo(data.mid(dataPtr - data.constData(), length));
//                        qDebug() << baseLlh.latitude << baseLlh.longitude << baseLlh.height << dataPtr - data.data() << length;
                        foundReferenceStationInfo = true;
                        emit baseStationPosition(baseLlh);
                    }
                }
                dataPtr++;
            }
        }
        emit rtcmData(data);
    });

    connect(&mTcpSocket, &QTcpSocket::connected, [this]{
        // If a stream is selected, we connected to an NTRIP server and potentially need to authenticate.
        if (mCurrentNtripConnectionInfo.stream.size() > 0) {
            QString msg;
            msg += "GET /" + mCurrentNtripConnectionInfo.stream + " HTTP/1.1\r\n";
            msg += "User-Agent: NTRIP " + mCurrentHost + "\r\n";

            if (mCurrentNtripConnectionInfo.user.size() > 0 || mCurrentNtripConnectionInfo.password.size() > 0) {
                QString authStr = mCurrentNtripConnectionInfo.user + ":" + mCurrentNtripConnectionInfo.password;
                QByteArray auth;
                auth.append(authStr);
                msg += "Authorization: Basic " + auth.toBase64() + "\r\n";
            }

            msg += "Accept: */*\r\nConnection: close\r\n";
            msg += "\r\n";

            mTcpSocket.write(msg.toLocal8Bit());
        }
    });
}

void RtcmClient::connectTcp(QString host, qint16 port)
{
    mCurrentHost = host;
    mCurrentPort = port;
    mCurrentNtripConnectionInfo = {"", "", ""};
    mTcpSocket.connectToHost(mCurrentHost, mCurrentPort);
}

void RtcmClient::connectNtrip(QString host, qint16 port, NtripConnectionInfo ntripConnectionInfo)
{
    mCurrentHost = host;
    mCurrentPort = port;
    mCurrentNtripConnectionInfo = ntripConnectionInfo;
    mTcpSocket.connectToHost(mCurrentHost, mCurrentPort);
}

bool RtcmClient::connectWithInfoFromFile(QString filePath)
{
    QFile rtcmServerInfoFile(filePath);
    if (!rtcmServerInfoFile.open(QIODevice::ReadOnly)) {
        qDebug() << "Warning: RtcmClient was unable to open" << filePath;
        return false;
    } else {
        QString serverName = QString(rtcmServerInfoFile.readLine()).trimmed();
        qint16 port = rtcmServerInfoFile.readLine().toShort();
        NtripConnectionInfo ntripConnectionInfo = {QString(rtcmServerInfoFile.readLine()).trimmed(),  // username
                                                   QString(rtcmServerInfoFile.readLine()).trimmed(),  // password
                                                   QString(rtcmServerInfoFile.readLine()).trimmed()}; // stream
//        qDebug() << serverName << port << ntripConnectionInfo.user << ntripConnectionInfo.password << ntripConnectionInfo.stream;

        connectNtrip(serverName, port, ntripConnectionInfo);
    }

    return isConnected();
}

bool RtcmClient::isConnected()
{
    return mTcpSocket.isOpen() && mTcpSocket.isReadable();
}

void RtcmClient::disconnect()
{
    if (isConnected())
        mTcpSocket.disconnectFromHost();
}

QString RtcmClient::getCurrentHost() const
{
    return mCurrentHost;
}

qint16 RtcmClient::getCurrentPort() const
{
    return mCurrentPort;
}

unsigned int RtcmClient::getbitu(const char *buff, int pos, int len) {
    unsigned int bits=0;
    int i;

    for (i = pos;i < pos + len;i++) {
        bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
    }

    return bits;
}

int RtcmClient::getbits(const char *buff, int pos, int len) {
    unsigned int bits = getbitu(buff, pos, len);

    if (len <= 0 || 32 <= len || !(bits & (1u << (len - 1)))) {
        return (int)bits;
    }

    return (int)(bits | (~0u << len)); // extend sign
}

double RtcmClient::getbits_38(const char *buff, int pos) {
    return (double)getbits(buff, pos, 32) * D(64.0) + getbitu(buff, pos + 32, 6);
}

llh_t RtcmClient::decodeLllhFromReferenceStationInfo(const QByteArray data)
{
    double p0 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    int bitIdx = 24 + 12;
    int staid;
    int itrf;
    llh_t llhResult = {0.0, 0.0, 0.0};

    if (bitIdx + 140 <= data.size() * 8) {
        staid = getbitu(data.constData(), bitIdx, 12); bitIdx+=12;
        itrf  = getbitu(data.constData(), bitIdx, 6);  bitIdx+= 6+4;
        p0    = getbits_38(data.constData(), bitIdx);  bitIdx+=38+2;
        p1    = getbits_38(data.constData(), bitIdx);  bitIdx+=38+2;
        p2    = getbits_38(data.constData(), bitIdx);


        p0 *= D(0.0001);
        p1 *= D(0.0001);
        p2 *= D(0.0001);


        // Convert ecef to llh
        double e2 = FE_WGS84 * (D(2.0) - FE_WGS84);
        double r2 = p0 * p0 + p1 * p1;
        double z = p2;
        double zk = 0.0;
        double sinp = 0.0;
        double v = RE_WGS84;

        while (fabs(z - zk) >= D(1E-4)) {
            zk = z;
            sinp = z / sqrt(r2 + z * z);
            v = RE_WGS84 / sqrt(D(1.0) - e2 * sinp * sinp);
            z = p2 + v * e2 * sinp;
        }

        llhResult.latitude = (r2 > D(1E-12) ? atan(z / sqrt(r2)) : (p2 > D(0.0) ? D_PI / D(2.0) : -D_PI / D(2.0))) * D(180.0) / D_PI;
        llhResult.longitude = (r2 > D(1E-12) ? atan2(p1, p0) : D(0.0)) * D(180.0) / D_PI;
        llhResult.height = sqrt(r2 + z * z) - v;
    }

    return llhResult;
}
