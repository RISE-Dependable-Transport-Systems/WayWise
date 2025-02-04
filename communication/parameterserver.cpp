/*
 *     Copyright 2023 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include <QDebug>
#include "parameterserver.h"
#include <QXmlStreamWriter>
#include <QFile>

ParameterServer* ParameterServer::mInstancePtr = nullptr;

ParameterServer::ParameterServer() {};

void ParameterServer::initialize()
{
    if(mInstancePtr)
        qDebug() << "Parameter server singleton already created";
    else
        mInstancePtr = new ParameterServer();
}

ParameterServer* ParameterServer::getInstance()
{
    if(!mInstancePtr)
        qDebug() << "Parameter server object has not been created";
    return mInstancePtr;
}

bool ParameterServer::updateIntParameter(std::string parameterName, int parameterValue)
{
    const std::lock_guard<std::mutex> lock(mMutex);
    auto search = mIntParameterToClassMapping.find(parameterName);

    if (search != mIntParameterToClassMapping.end()) {
        std::function<void(int)> setParameterFunction = search->second.first;
        setParameterFunction(parameterValue);
        return true;
    } else
        return false;
}

bool ParameterServer::updateFloatParameter(std::string parameterName, float parameterValue)
{
    const std::lock_guard<std::mutex> lock(mMutex);
    auto search = mFloatParameterToClassMapping.find(parameterName);

    if (search != mFloatParameterToClassMapping.end()) {
        std::function<void(float)> setParameterFunction = search->second.first;
        setParameterFunction(parameterValue);
        return true;
    } else
        return false;
}

void ParameterServer::provideIntParameter(std::string parameterName, std::function<void(int)> setClassParameterFunction, std::function<int(void)> getClassParameterFunction)
{
    const std::lock_guard<std::mutex> lock(mMutex);
    mIntParameterToClassMapping.insert_or_assign(parameterName, std::make_pair(setClassParameterFunction, getClassParameterFunction));
};

void ParameterServer::provideFloatParameter(std::string parameterName, std::function<void(float)> setClassParameterFunction, std::function<float(void)> getClassParameterFunction)
{
    const std::lock_guard<std::mutex> lock(mMutex);
    mFloatParameterToClassMapping.insert_or_assign(parameterName, std::make_pair(setClassParameterFunction, getClassParameterFunction));
};

void ParameterServer::saveParametersToXmlFile(QString filename)
{
    const std::lock_guard<std::mutex> lock(mMutex);
    QFile parameterFile(filename);

    if (!parameterFile.open(QIODevice::WriteOnly)) {
        qDebug() << "Failed to open file, could not save parameters";
        return;
    }

    QXmlStreamWriter stream(&parameterFile);
    stream.setCodec("UTF-8");
    stream.setAutoFormatting(true);
    stream.writeStartDocument();

    for (const auto& parameter : mIntParameterToClassMapping) {
        std::string parameterName = parameter.first;
        int parameterValue = parameter.second.second();
        stream.writeTextElement(QString::fromStdString(parameterName), QString::number(parameterValue));
    }

    for (const auto& parameter : mFloatParameterToClassMapping) {
        std::string parameterName = parameter.first;
        float parameterValue = parameter.second.second();
        stream.writeTextElement(QString::fromStdString(parameterName), QString::number(parameterValue));
    }

    stream.writeEndElement();
    stream.writeEndDocument();
    parameterFile.close();
};

ParameterServer::AllParameters ParameterServer::getAllParameters()
{
    ParameterServer::IntParameter intParameter;
    ParameterServer::FloatParameter floatParameter;
    ParameterServer::AllParameters allParameters;

    for (const auto& parameter : mIntParameterToClassMapping) {
        intParameter.name = parameter.first;
        intParameter.value = parameter.second.second();
        allParameters.intParameters.push_back(intParameter);
    }

    for (const auto& parameter : mFloatParameterToClassMapping) {
        floatParameter.name = parameter.first;
        floatParameter.value = parameter.second.second();
        allParameters.floatParameters.push_back(floatParameter);
    }

    return allParameters;
}
