#include "servocontroller.h"

bool ServoController::getInvertOutput() const
{
    return mInvertOutput;
}

void ServoController::setInvertOutput(bool invertOutput)
{
    mInvertOutput = invertOutput;
}

double ServoController::getServoRange() const
{
    return mServoRange;
}

void ServoController::setServoRange(double servoRange)
{
    mServoRange = servoRange;
}

double ServoController::getServoCenter() const
{
    return mServoCenter;
}

void ServoController::setServoCenter(double servoCenter)
{
    mServoCenter = servoCenter;
}
