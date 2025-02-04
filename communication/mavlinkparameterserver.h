/*
 *     Copyright 2023 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

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
    static void initialize(std::shared_ptr<mavsdk::ServerComponent> serverComponent);
    virtual void provideIntParameter(std::string parameterName, std::function<void(int)> setClassParameterFunction, std::function<int(void)> getClassParameterFunction) override;
    virtual void provideFloatParameter(std::string parameterName, std::function<void(float)> setClassParameterFunction, std::function<float(void)> getClassParameterFunction) override;
    virtual void saveParametersToXmlFile(QString filename) override;

protected:
    MavlinkParameterServer(std::shared_ptr<mavsdk::ServerComponent> serverComponent);
    ~MavlinkParameterServer(){};

private:
    mavsdk::ParamServer *mMavsdkParamServer;
};

#endif // PARAMETERSERVER_H
