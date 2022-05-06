/*
    Copyright 2016 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "tcpserversimple.h"
#include <QDebug>

TcpServerSimple::TcpServerSimple(QObject *parent) : QObject(parent)
{
    mTcpServer = new QTcpServer(this);
    mPacket = new Packet(this);
    mTcpSocket = 0;
    mUsePacket = false;

    connect(mTcpServer, &QTcpServer::newConnection, this, &TcpServerSimple::newTcpConnection);
    connect(mPacket, &Packet::dataToSend, this, &TcpServerSimple::dataToSend);
}

bool TcpServerSimple::startServer(int port)
{
    if (!mTcpServer->listen(QHostAddress::Any,  port)) {
        return false;
    }

    return true;
}

void TcpServerSimple::stopServer()
{
    mTcpServer->close();

    if (mTcpSocket) {
        mTcpSocket->close();
        delete mTcpSocket;
        mTcpSocket = 0;
        emit connectionChanged(false);
    }
}

bool TcpServerSimple::sendData(const QByteArray &data)
{
    bool res = false;

    if (mTcpSocket) {
        mTcpSocket->write(data);
        res = true;
    }

    return res;
}

QString TcpServerSimple::errorString()
{
    return mTcpServer->errorString();
}

Packet *TcpServerSimple::packet()
{
    return mPacket;
}

void TcpServerSimple::newTcpConnection()
{
    QTcpSocket *socket = mTcpServer->nextPendingConnection();
    socket->setSocketOption(QAbstractSocket::LowDelayOption, true);

    if (mTcpSocket) {
        socket->close();
        delete socket;
    } else {
        mTcpSocket = socket;

        if (mTcpSocket) {
            connect(mTcpSocket, &QTcpSocket::readyRead, this, &TcpServerSimple::tcpInputDataAvailable);
            connect(mTcpSocket, &QTcpSocket::disconnected, this, &TcpServerSimple::tcpInputDisconnected);
            connect(mTcpSocket, QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::error), this, &TcpServerSimple::tcpInputError);

            emit connectionChanged(true);
        }
    }
}

void TcpServerSimple::tcpInputDisconnected()
{
    mTcpSocket->deleteLater();
    mTcpSocket = 0;
    emit connectionChanged(false);
}

void TcpServerSimple::tcpInputDataAvailable()
{
    QByteArray data = mTcpSocket->readAll();
    emit dataRx(data);

    if (mUsePacket) {
        mPacket->processData(data);
    }
}

void TcpServerSimple::tcpInputError(QAbstractSocket::SocketError socketError)
{
    (void)socketError;
    mTcpSocket->close();
    delete mTcpSocket;
    mTcpSocket = 0;
    emit connectionChanged(false);
}

void TcpServerSimple::dataToSend(QByteArray &data)
{
    sendData(data);
}

bool TcpServerSimple::usePacket() const
{
    return mUsePacket;
}

void TcpServerSimple::setUsePacket(bool usePacket)
{
    mUsePacket = usePacket;
}
