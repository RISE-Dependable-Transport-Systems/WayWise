#include "depthaicamera.h"
#include <QDebug>

DepthAiCamera::DepthAiCamera()
{
    // Connect to camera stream
    mJsonParser.connectToHost(QHostAddress::LocalHost, 8070);
    QObject::connect(&mJsonParser, &JsonStreamParserTcp::gotJsonArray, this, &DepthAiCamera::cameraInput);
}

void DepthAiCamera::cameraInput(const QJsonArray& jsonArray)
{
    // 3D position x,y,z in meters from the camera.

    // Objects detected, save only the closest one
    double closeObject = 1000;
    for (int i=0; i<jsonArray.size(); i++) {
        if (closeObject > jsonArray.at(i).toObject().value("depth_z").toDouble()) {
            mCameraData.setX(jsonArray.at(i).toObject().value("depth_x").toDouble());
            mCameraData.setY(jsonArray.at(i).toObject().value("depth_z").toDouble());
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

    emit closestObject(mCameraData);

//    qDebug() << jsonArray
//             << "\nsize:" << jsonArray.size();
}
