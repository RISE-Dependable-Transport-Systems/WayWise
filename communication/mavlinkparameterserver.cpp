/*
 *     Copyright 2023 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#include "mavlinkparameterserver.h"
#include <QXmlStreamWriter>
#include <QFile>
#include <QDebug>

MavlinkParameterServer::MavlinkParameterServer(std::shared_ptr<mavsdk::ServerComponent> serverComponent)
{
    mMavsdkParamServer = new mavsdk::ParamServer(serverComponent);

    // These are needed for MAVSDK at the moment
    mMavsdkParamServer->provide_param_int("CAL_ACC0_ID", 1);
    mMavsdkParamServer->provide_param_int("CAL_GYRO0_ID", 1);
    mMavsdkParamServer->provide_param_int("CAL_MAG0_ID", 1);
    mMavsdkParamServer->provide_param_int("SYS_HITL", 0);
    mMavsdkParamServer->provide_param_int("MIS_TAKEOFF_ALT", 0);
}

void MavlinkParameterServer::initialize(std::shared_ptr<mavsdk::ServerComponent> serverComponent)
{
    if(mInstancePtr)
        qDebug() << "Parameter server singleton already created";
    else
        mInstancePtr = new MavlinkParameterServer(serverComponent);
}

/**
 * By convention, every parameter in a group should share the same (meaningful) string prefix followed by an underscore.
 *
 * @param parameterName
 * Parameter names must be no more than 16 ASCII characters
 */
void MavlinkParameterServer::provideIntParameter(std::string parameterName, std::function<void(int)> setClassParameterFunction, std::function<int(void)> getClassParameterFunction)
{
    const std::lock_guard<std::mutex> lock(mMutex);
    mIntParameterToClassMapping.insert_or_assign(parameterName, std::make_pair(setClassParameterFunction, getClassParameterFunction));
    mMavsdkParamServer->provide_param_int(parameterName, getClassParameterFunction());
};

void MavlinkParameterServer::provideFloatParameter(std::string parameterName, std::function<void(float)> setClassParameterFunction, std::function<float(void)> getClassParameterFunction)
{
    const std::lock_guard<std::mutex> lock(mMutex);
    mFloatParameterToClassMapping.insert_or_assign(parameterName, std::make_pair(setClassParameterFunction, getClassParameterFunction));
    mMavsdkParamServer->provide_param_float(parameterName, getClassParameterFunction());
};

void MavlinkParameterServer::saveParametersToXmlFile(QString filename)
{
    const std::lock_guard<std::mutex> lock(mMutex);
    mavsdk::ParamServer::AllParams parameters = mMavsdkParamServer->retrieve_all_params();
    QFile parameterFile(filename);

    if (!parameterFile.open(QIODevice::WriteOnly)) {
        qDebug() << "Failed to open file, could not save parameters";
        return;
    }

    QXmlStreamWriter stream(&parameterFile);
    stream.setCodec("UTF-8");
    stream.setAutoFormatting(true);
    stream.writeStartDocument();

    for (const auto& vehicleParameter : parameters.int_params) {
        stream.writeTextElement(QString::fromStdString(vehicleParameter.name), QString::number(vehicleParameter.value));
    }
    for (const auto& vehicleParameter : parameters.float_params) {
        stream.writeTextElement(QString::fromStdString(vehicleParameter.name), QString::number(vehicleParameter.value));
    }
    for (const auto& vehicleParameter : parameters.custom_params) {
        stream.writeTextElement(QString::fromStdString(vehicleParameter.name), QString::fromStdString(vehicleParameter.value));
    }

    stream.writeEndElement();
    stream.writeEndDocument();
    parameterFile.close();
};
