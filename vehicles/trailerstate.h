/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 * 
 * Specialisation of ObjectState for tailer state 
 */

#ifndef TRAILERSTATE_H
#define TRAILERSTATE_H


#include <QObject>
#include "vehicles/objectstate.h"

#ifdef QT_GUI_LIB
#include <QPainter>
#endif

class TrailerState : public ObjectState
{
    Q_OBJECT
public:

    TrailerState(ObjectID_t id = 0, Qt::GlobalColor color = Qt::white);

    double getLength() const { return mLength; }
    void setLength(double length) { mLength = length; }
    double getWidth() const { return mWidth; }
    void setWidth(double width) { mWidth = width; }


private:
    double mLength; // [m]
    double mWidth; // [m]

};

#endif // TRAILERSTATE_H