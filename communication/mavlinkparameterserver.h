#ifndef MAVLINKPARAMETERSERVER_H
#define MAVLINKPARAMETERSERVER_H

#include <QObject>
#include <mavsdk/server_component.h>
#include <mavsdk/plugins/param_server/param_server.h>
#include "communication/parameterserver.h"

class MavlinkParameterServer : public ParameterServer
{
    Q_OBJECT
public:
    explicit MavlinkParameterServer(std::shared_ptr<mavsdk::ServerComponent> serverComponent);
    void provideParameter(std::string parameterName);
    ParameterServer::AllParameters retreiveAllParameters();
    virtual void saveParametersToXmlFile() override;

private:
    mavsdk::ParamServer *mMavsdkParamServer;
};

#endif // PARAMETERSERVER_H
