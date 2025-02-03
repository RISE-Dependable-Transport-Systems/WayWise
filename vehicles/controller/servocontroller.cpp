/*
 *     Copyright 2021 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
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
