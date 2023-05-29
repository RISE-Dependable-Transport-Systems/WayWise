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

void MavlinkParameterServer::provideParameter(std::string parameterName)
{
    auto getParameterFunction = mParameterToClassMapping.find(parameterName)->second.second;
    mMavsdkParamServer->provide_param_float(parameterName, getParameterFunction());
};

ParameterServer::AllParameters MavlinkParameterServer::retreiveAllParameters()
{
     mavsdk::ParamServer::AllParams mavsdkVehicleParameters = mMavsdkParamServer->retrieve_all_params();
     ParameterServer::IntParameter intParameter;
     ParameterServer::FloatParameter floatParameter;
     ParameterServer::CustomParameter customParameter;
     ParameterServer::AllParameters allParameters;

     for (const auto& vehicleParameter : mavsdkVehicleParameters.int_params) {
         intParameter.name = vehicleParameter.name;
         intParameter.value = vehicleParameter.value;
         allParameters.intParameters.push_back(intParameter);
     }
     for (const auto& vehicleParameter : mavsdkVehicleParameters.float_params) {
         floatParameter.name = vehicleParameter.name;
         floatParameter.value = vehicleParameter.value;
         allParameters.floatParameters.push_back(floatParameter);
     }
     for (const auto& vehicleParameter : mavsdkVehicleParameters.custom_params) {
         customParameter.name = vehicleParameter.name;
         customParameter.value = vehicleParameter.value;
         allParameters.customParameters.push_back(customParameter);
     }
    return allParameters;
}

void MavlinkParameterServer::saveParametersToXmlFile()
{
    mavsdk::ParamServer::AllParams parameters = mMavsdkParamServer->retrieve_all_params();
    QFile parameterFile("vehicle_parameters.xml");

    if (!parameterFile.open(QIODevice::WriteOnly))
        qDebug() << "Could not save parameters";

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
