#include "tracemodule.h"

TraceModule::TraceModule()
{
    // Trace simulated, GNSS and fused by default
    memset(mTraceModuleState.traceActiveForPosType, 0, sizeof(mTraceModuleState.traceActiveForPosType));
    setTraceActiveForPosType(PosType::simulated, true);
    setTraceColorForPosType(PosType::simulated, Qt::blue);
    setTraceActiveForPosType(PosType::GNSS, true);
    setTraceColorForPosType(PosType::GNSS, Qt::magenta);
    setTraceActiveForPosType(PosType::fused, true);
    setTraceColorForPosType(PosType::fused, Qt::red);

    connect(&mTraceSampleTimer, &QTimer::timeout, [this](){
        if (mTraceModuleState.currentTraceIndex < 0)
            return;

        for (int currentPosTypeInt = 0; currentPosTypeInt < (int)PosType::_LAST_; currentPosTypeInt++) {
            if (mTraceModuleState.traceActiveForPosType[currentPosTypeInt]) {
                while (mTraceModuleState.currentTraceIndex >= mTraceListPerPosType[currentPosTypeInt].size())
                    mTraceListPerPosType[currentPosTypeInt].append(QList<PosPoint>());


                if (mTraceListPerPosType[currentPosTypeInt][mTraceModuleState.currentTraceIndex].size() == 0
                    || QLineF(mTraceListPerPosType[currentPosTypeInt][mTraceModuleState.currentTraceIndex].last().getPoint(),
                              mTraceModuleState.currentTraceVehicle->getPosition((PosType)currentPosTypeInt).getPoint()).length() > mTraceModuleState.minTraceSampleDistance)
                    mTraceListPerPosType[currentPosTypeInt][mTraceModuleState.currentTraceIndex].
                            append(mTraceModuleState.currentTraceVehicle->getPosition((PosType)currentPosTypeInt));
            }
        }
    });
}


void TraceModule::processPaint(QPainter &painter, int width, int height, bool highQuality, QTransform drawTrans, QTransform txtTrans, double scale)
{
    QPen pen;
    pen.setWidthF(7.5/scale);
    painter.setTransform(drawTrans);

    if (mTraceModuleState.currentTraceIndex < 0)
        return;

    for (int currentPosTypeInt = 0; currentPosTypeInt < (int)PosType::_LAST_; currentPosTypeInt++) {
        if (mTraceModuleState.traceActiveForPosType[currentPosTypeInt]) {
            pen.setColor(mTraceModuleState.traceColorForPosType[currentPosTypeInt]);
            painter.setPen(pen);
            if (mTraceModuleState.currentTraceIndex < mTraceListPerPosType[currentPosTypeInt].size()) {
                for (int i = 1; i < mTraceListPerPosType[currentPosTypeInt][mTraceModuleState.currentTraceIndex].size(); i++) {
                    painter.drawLine(mTraceListPerPosType[currentPosTypeInt][mTraceModuleState.currentTraceIndex][i-1].getPointMm(),
                            mTraceListPerPosType[currentPosTypeInt][mTraceModuleState.currentTraceIndex][i].getPointMm());
                }
            }
        }
    }
}

bool TraceModule::processMouse(bool isPress, bool isRelease, bool isMove, bool isWheel, QPoint widgetPos, PosPoint mapPos, double wheelAngleDelta, Qt::KeyboardModifiers keyboardModifiers, Qt::MouseButtons mouseButtons, double scale)
{
    return false;
}

void TraceModule::setTraceActiveForPosType(PosType type, bool active)
{
    mTraceModuleState.traceActiveForPosType[(int)type] = active;
}

void TraceModule::setTraceColorForPosType(PosType type, QColor color)
{
    mTraceModuleState.traceColorForPosType[(int)type] = color;
}

void TraceModule::setCurrentTraceVehicle(QSharedPointer<VehicleState> traceVehicle)
{
    mTraceModuleState.currentTraceVehicle = traceVehicle;
}

void TraceModule::startTrace(int traceIndex)
{
    if (mTraceModuleState.currentTraceVehicle == nullptr)
        return;

    if (traceIndex >= 0)
        mTraceModuleState.currentTraceIndex = traceIndex;

    if (mTraceModuleState.currentTraceIndex >= 0)
        mTraceSampleTimer.start(mTraceSampleTimerPeriod_ms);
}

void TraceModule::setCurrentTraceIndex(int traceIndex)
{
    if (traceIndex >= 0)
        mTraceModuleState.currentTraceIndex = traceIndex;
}

int TraceModule::getCurrentTraceIndex()
{
    return mTraceModuleState.currentTraceIndex;
}

void TraceModule::stopTrace()
{
    mTraceSampleTimer.stop();
}

void TraceModule::clearTraceIndex(int traceIndex)
{
    if (traceIndex >= 0)
        for (int currentPosTypeInt = 0; currentPosTypeInt < (int)PosType::_LAST_; currentPosTypeInt++)
            if (mTraceModuleState.currentTraceIndex < mTraceListPerPosType[currentPosTypeInt].size())
                mTraceListPerPosType[currentPosTypeInt][mTraceModuleState.currentTraceIndex].clear();
}

void TraceModule::setTraceSamplePeriod(int traceSampleTimerPeriod_ms)
{
    mTraceSampleTimerPeriod_ms = traceSampleTimerPeriod_ms;
}
