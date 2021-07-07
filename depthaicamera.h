#ifndef DEPTHAICAMERA_H
#define DEPTHAICAMERA_H

#include <QObject>
#include "pospoint.h"
#include "jsonstreamparsertcp.h"

class DepthAiCamera : public QObject
{
    Q_OBJECT
public:
    DepthAiCamera();

signals:
    void closestObject(const PosPoint &obj);

private:
    JsonStreamParserTcp mJsonParser;

    PosPoint mCameraData;
    void cameraInput(const QJsonArray& jsonArray);
};

#endif // DEPTHAICAMERA_H
