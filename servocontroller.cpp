#include "servocontroller.h"

bool ServoController::getInvertOutput() const
{
    return mInvertOutput;
}

void ServoController::setInvertOutput(bool invertOutput)
{
    mInvertOutput = invertOutput;
}
