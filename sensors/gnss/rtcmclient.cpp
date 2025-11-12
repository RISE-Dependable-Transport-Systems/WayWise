/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "rtcmclient.h"
#include <QFile>

RtcmClient::RtcmClient(QObject *parent) : QObject(parent)
{
    connect(&mTcpSocket, &QTcpSocket::readyRead, [this]{
        QByteArray data =  mTcpSocket.readAll();
//        qDebug() << data;

        // If first reply contains headers, parse them to check HTTP status / ICY
        if (!mSkippedFirstReply) {
            mSkippedFirstReply = true;

            // Convert to string safely (headers are ASCII). If binary, this will truncate at \0
            QString headerStr = QString::fromLatin1(data);
            // qDebug() << "First reply (as text):\n" << headerStr;

            // Useful checks:
            if (headerStr.startsWith("ICY 200") || headerStr.contains("200 OK")) {
                qDebug() << "RtcmClient: NTRIP server responded OK.";
            } else {
                qWarning() << "RtcmClient: NTRIP server initial reply not 200. It might be an error or sourcetable.";
                qDebug() << "RtcmClient: NTRIP server initial reply:" << headerStr;
            }

            int headerEnd = headerStr.indexOf("\r\n\r\n");
            if (headerEnd >= 0) {
                QByteArray remainder = data.mid(headerEnd + 4);
                if (!remainder.isEmpty()) {
                    // handle remainder (may contain first RTCM bytes)
                    // fall-through to RTCM scanning by replacing `data` with remainder
                    data = remainder;
                } else {
                    // no more data in this chunk after headers, return to wait for RTCM
                    return;
                }
            } else {
                // Headers not complete yet — return and wait for more bytes
                return;
            }
        }

        // Now `data` should be pointing at RTCM binary stream; try to find RTCM messages
        if (!mFoundReferenceStationInfo) {
            const char* dataPtr = data.constData();
            auto dataEnd = data.constEnd();

            while ((dataPtr = std::find(dataPtr, dataEnd, RTCM3_PREAMBLE)) != dataEnd) {
                if (dataPtr + 5 < dataEnd) {
                    int length = getbitu(dataPtr, 14, 10) + 1; // bytes incl. CRC
                    int type = getbitu(dataPtr, 24, 12);
                    if (dataPtr + length < dataEnd) {
                        if (type == 1005 || type == 1006) {
                            QByteArray msgBytes = data.mid(dataPtr - data.constData(), length);
                            llh_t baseLlh = decodeLllhFromReferenceStationInfo(msgBytes);
                            qDebug() << "RtcmClient got base station position:" << baseLlh.latitude << baseLlh.longitude << baseLlh.height;
                            mFoundReferenceStationInfo = true;
                            emit baseStationPosition(baseLlh);
                        }
                    }
                }
                ++dataPtr;
            }
        }

        emit rtcmData(data);
    });

    connect(&mTcpSocket, &QTcpSocket::connected, [this]{
        if (mCurrentNtripConnectionInfo.stream.size() == 0) {
            qDebug() << "No mountpoint/stream specified.";
            return;
        }

        QString msg;
        // Use HTTP/1.0 and include Host header
        msg += "GET /" + mCurrentNtripConnectionInfo.stream + " HTTP/1.0\r\n";
        msg += "Host: " + mCurrentHost + "\r\n";
        msg += "User-Agent: NTRIP client/1.0\r\n";

        if (!mCurrentNtripConnectionInfo.user.isEmpty() || !mCurrentNtripConnectionInfo.password.isEmpty()) {
            QString authStr = mCurrentNtripConnectionInfo.user + ":" + mCurrentNtripConnectionInfo.password;
            QByteArray auth;
            auth.append(authStr.toLocal8Bit());
            msg += "Authorization: Basic " + auth.toBase64() + "\r\n";
        }

        msg += "Accept: */*\r\n\r\n"; // no Connection: close for now

        // qDebug() << msg;

        mTcpSocket.write(msg.toLocal8Bit());
    });

    // connect error and state for debugging (call once, e.g. in constructor)
    connect(&mTcpSocket, &QTcpSocket::errorOccurred, [this](QAbstractSocket::SocketError err){
        qWarning() << "RtcmClient socket error:" << err << mTcpSocket.errorString();
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
        qint16 port = QString(rtcmServerInfoFile.readLine()).trimmed().toShort();
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

void RtcmClient::forwardNmeaGgaToServer(const QByteArray &nmeaGgaStr)
{
    // Send NMEA GGA to NTRIP/RTCM server until we got reference station information.
    // Some NTRIP servers will not start sending RTCM unless they got NMEA GGA.
    if (isConnected() && mSkippedFirstReply)
        mTcpSocket.write(nmeaGgaStr);
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
    int staid; Q_UNUSED(staid)
    int itrf; Q_UNUSED(itrf)
    llh_t llhResult = {0.0, 0.0, 0.0};

//    if (bitIdx + 140 <= data.size() * 8) {
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
//    }

    return llhResult;
}
