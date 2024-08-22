/*
 *     Copyright 2023 Rickard Häll      rickard.hall@ri.se
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

bool ParameterServer::updateParameter(std::string parameterName, float parameterValue)
{
    const std::lock_guard<std::mutex> lock(mMutex);
    auto search = mParameterToClassMapping.find(parameterName);

    if (search != mParameterToClassMapping.end()) {
        auto setParameterFunction = search->second.first;
        setParameterFunction(parameterValue);
        return true;
    } else
        return false;
}

void ParameterServer::provideParameter(std::string parameterName, std::function<void(float)> setClassParameterFunction, std::function<float(void)> getClassParameterFunction)
{
    const std::lock_guard<std::mutex> lock(mMutex);
    mParameterToClassMapping.insert_or_assign(parameterName, std::make_pair(setClassParameterFunction, getClassParameterFunction));
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

    for (const auto& parameter : mParameterToClassMapping) {
        auto parameterName = parameter.first;
        auto parameterValue = parameter.second.second();
        stream.writeTextElement(QString::fromStdString(parameterName), QString::number(parameterValue));
    }

    stream.writeEndElement();
    stream.writeEndDocument();
    parameterFile.close();
};

ParameterServer::AllParameters ParameterServer::getAllParameters()
{
    ParameterServer::FloatParameter floatParameter;
    ParameterServer::AllParameters allParameters;

    for (const auto& parameter : mParameterToClassMapping) {
        floatParameter.name = parameter.first;
        floatParameter.value = parameter.second.second();
        allParameters.floatParameters.push_back(floatParameter);
    }

    return allParameters;
}
