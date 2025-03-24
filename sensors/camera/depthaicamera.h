/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Class to parse a JSON stream with object detections (incl. depth information) from DepthAI
 */

#ifndef DEPTHAICAMERA_H
#define DEPTHAICAMERA_H

#include <QObject>
#include <QTimer>
#include "core/pospoint.h"
#include "communication/jsonstreamparsertcp.h"

class DepthAiCamera : public QObject
{
    Q_OBJECT
public:
    DepthAiCamera();

signals:
    void closestObject(const PosPoint &obj);
    void brakeSignal(const QString& msg);

private slots:
    void checkConnection();

private:
    JsonStreamParserTcp mJsonParser;

    PosPoint mCameraData;
    void cameraInput(const QString& tcpMsg);
    QTimer mTimer;
};

#endif // DEPTHAICAMERA_H
