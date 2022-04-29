/*
 *     Copyright 2021 Rickard Häll      rickard.hall@ri.se
 *               2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 * 
 * Class to parse a JSON stream with object detections (incl. depth information) from DepthAI
 */

#ifndef DEPTHAICAMERA_H
#define DEPTHAICAMERA_H

#include <QObject>
#include "core/pospoint.h"
#include "communication/jsonstreamparsertcp.h"

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
