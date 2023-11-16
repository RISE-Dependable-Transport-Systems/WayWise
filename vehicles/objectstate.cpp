/*
 *     Copyright 2012 Benjamin Vedder   benjamin@vedder.se
 *               2020 Marvin Damschen   marvin.damschen@ri.se
 *               2021 Lukas Wikander    lukas.wikander@astazero.com
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#include "objectstate.h"
#include <QDebug>
#include <QTextStream>

ObjectState::ObjectState(ObjectID_t id, Qt::GlobalColor color)
{
    setId(id, true);
    setColor(color);
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
    mPosition = point;
    emit positionUpdated();
}

void ObjectState::setDrawStatusText(bool drawStatusText)
{
    mDrawStatusText = drawStatusText;
}

bool ObjectState::getDrawStatusText() const
{
    return mDrawStatusText;
}
