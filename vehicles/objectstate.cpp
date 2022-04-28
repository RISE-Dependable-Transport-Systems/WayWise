/*
    Copyright 2012 Benjamin Vedder	benjamin@vedder.se
              2020 Marvin Damschen  marvin.damschen@ri.se
              2021 Lukas Wikander	lukas.wikander@astazero.com

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

#include "objectstate.h"
#include <QDebug>

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
        mName.sprintf("Vehicle %d", mId);
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
