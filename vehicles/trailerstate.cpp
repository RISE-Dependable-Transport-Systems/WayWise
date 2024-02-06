/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 */

#include "trailerstate.h"


TrailerState::TrailerState(ObjectState::ObjectID_t id, Qt::GlobalColor color) : ObjectState (id, color)
{    

    mLength = 1;
    mWidth = 0.3;
    
}