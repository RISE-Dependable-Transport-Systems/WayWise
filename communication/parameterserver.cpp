/*
 *     Copyright 2023 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include <QDebug>
#include "parameterserver.h"

ParameterServer* ParameterServer::mInstancePtr = nullptr;

ParameterServer::ParameterServer() {};

ParameterServer* ParameterServer::getInstance()
{
    if(!mInstancePtr)
        qDebug() << "Parameter server object has not been created";
    return mInstancePtr;
}

void ParameterServer::updateParameter(std::string parameterName, float parameterValue)
{
    const std::lock_guard<std::mutex> lock(mMutex);
    auto setParameterFunction = mParameterToClassMapping.find(parameterName)->second.first;
    setParameterFunction(parameterValue);
}
