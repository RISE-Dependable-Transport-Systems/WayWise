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

void ParameterServer::updateParameter(std::string parameterName, float parameterValue)
{
    const std::lock_guard<std::mutex> lock(mMutex);
    auto setParameterFunction = mParameterToClassMapping.find(parameterName)->second.first;
    setParameterFunction(parameterValue);
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

    if (!parameterFile.open(QIODevice::WriteOnly))
        qDebug() << "Could not save parameters";

    QXmlStreamWriter stream(&parameterFile);
    stream.setCodec("UTF-8");
    stream.setAutoFormatting(true);
    stream.writeStartDocument();

    for (const auto& vehicleParameter : mParameterToClassMapping) {
        auto parameterName = vehicleParameter.first;
        auto ParameterValue = vehicleParameter.second.second();
        stream.writeTextElement(QString::fromStdString(parameterName), QString::number(ParameterValue));
    }

    stream.writeEndElement();
    stream.writeEndDocument();
    parameterFile.close();
};
