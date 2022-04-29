/*
 *     Copyright 2021 Rickard Häll      rickard.hall@ri.se
 *               2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 * 
 * Class to parse a JSON stream with object detections (incl. depth information) from DepthAI
 */
#include "depthaicamera.h"
#include <QDebug>

DepthAiCamera::DepthAiCamera()
{
    // Connect to camera stream
    mJsonParser.connectToHost(QHostAddress::LocalHost, 8070);
    QObject::connect(&mJsonParser, &JsonStreamParserTcp::gotJsonArray, this, &DepthAiCamera::cameraInput);
    QObject::connect(&mJsonParser, &JsonStreamParserTcp::connectionError, [](QTcpSocket::SocketError error){
        qDebug() << "Info: DepthAiCamera not connected, got" << error;
    });
}

void DepthAiCamera::cameraInput(const QJsonArray& jsonArray)
{
    // 3D position x,y,z in meters from the camera.

    // Objects detected, save only the closest one
    double closeObject = std::numeric_limits<double>::max();
    for (int i=0; i<jsonArray.size(); i++) {
        if (closeObject > jsonArray.at(i).toObject().value("depth_z").toDouble()) {
            mCameraData.setX(jsonArray.at(i).toObject().value("depth_z").toDouble());
            mCameraData.setY(-jsonArray.at(i).toObject().value("depth_x").toDouble());
            mCameraData.setHeight(jsonArray.at(i).toObject().value("depth_y").toDouble());
            closeObject = mCameraData.getY();
        }
    }

    // No objects detected
    if (jsonArray.size() == 0) {
        mCameraData.setX(0);
        mCameraData.setY(0);
        mCameraData.setHeight(0);
    }

    mCameraData.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
    emit closestObject(mCameraData);

//    qDebug() << jsonArray
//             << "\nsize:" << jsonArray.size();
}
