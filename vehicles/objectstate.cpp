/*
 *     Copyright 2021 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#include "objectstate.h"
#include <QDebug>
#include <QTextStream>

ObjectState::ObjectState(ObjectID_t id, Qt::GlobalColor color)
{
    setId(id, true);
    setColor(color);
    mTime = QTime();

    for (int i = 0; i < (int)PosType::_LAST_; i++)
        switch((PosType) i) {
            case PosType::simulated: mPositionBySource[i].setType(PosType::simulated); break;
            case PosType::fused: mPositionBySource[i].setType(PosType::fused); break;
            case PosType::odom: mPositionBySource[i].setType(PosType::odom); break;
            case PosType::IMU: mPositionBySource[i].setType(PosType::IMU); break;
            case PosType::GNSS: mPositionBySource[i].setType(PosType::GNSS); break;
            case PosType::UWB: mPositionBySource[i].setType(PosType::UWB); break;
            case PosType::_LAST_: qDebug() << "This should not have happended."; break;
        }
}

void ObjectState::setId(int id, bool changeName)
{
    mId = id;
    if (changeName) {
        mName = "";
        QTextStream(&mName) << "Vehicle " << mId;
    }
}


void ObjectState::setPosition(PosPoint &point)
{
    mPositionBySource[(int)point.getType()] = point;

    emit positionUpdated(point.getType());
}

void ObjectState::setDrawStatusText(bool drawStatusText)
{
    mDrawStatusText = drawStatusText;
}

bool ObjectState::getDrawStatusText() const
{
    return mDrawStatusText;
}

PosPoint ObjectState::getPosition(PosType type) const
{
    return mPositionBySource[(int)type];
}
