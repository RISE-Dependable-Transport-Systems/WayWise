#ifndef TRACEMODULE_H
#define TRACEMODULE_H

#include "userinterface/map/mapwidget.h"
#include "core/pospoint.h"

class TraceModule : public MapModule
{
public:
    TraceModule();

    // MapModule interface
    virtual void processPaint(QPainter &painter, int width, int height, bool highQuality, QTransform drawTrans, QTransform txtTrans, double scale) override;
    virtual bool processMouse(bool isPress, bool isRelease, bool isMove, bool isWheel, QPoint widgetPos, PosPoint mapPos, double wheelAngleDelta, Qt::KeyboardModifiers keyboardModifiers, Qt::MouseButtons mouseButtons, double scale) override;

    void setTraceActiveForPosType(PosType type, bool active);
    void setTraceColorForPosType(PosType type, QColor color);
    void setCurrentTraceVehicle(QSharedPointer<VehicleState> traceVehicle);
    void startTrace(int traceIndex = -1);
    void setCurrentTraceIndex(int traceIndex);
    int getCurrentTraceIndex();
    void stopTrace();
    void clearTraceIndex(int traceIndex);
    void setTraceSamplePeriod(int traceSamplePeriod_ms);

private:
    struct {
        int currentTraceIndex = -1;
        bool traceActiveForPosType[(int)PosType::_LAST_];
        QColor traceColorForPosType[(int)PosType::_LAST_];
        QSharedPointer<VehicleState> currentTraceVehicle = nullptr;
        double minTraceSampleDistance = 0.1; // [m]
    } mTraceModuleState;

    QTimer mTraceSampleTimer;
    int mTraceSampleTimerPeriod_ms = 100;
    QList<QList<PosPoint>> mTraceListPerPosType[(int)PosType::_LAST_];

};

#endif // TRACEMODULE_H
