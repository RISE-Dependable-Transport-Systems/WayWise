#ifndef PARAMETERSERVER_H
#define PARAMETERSERVER_H

#include <QObject>
#include <mavsdk/server_component.h>
#include <mavsdk/plugins/param_server/param_server.h>
#include <unordered_map>

class ParameterServer : public QObject
{
    Q_OBJECT
public:
    explicit ParameterServer(std::shared_ptr<mavsdk::ServerComponent> serverComponent);
    void updateParameter(std::string parameterName, float parameterValue);
    void provideParameter(std::string parameterName, std::function<void(float)> setClassParameterFunction, std::function<float(void)> getClassParameterFunction);
    void saveParametersToXmlFile();

private:
    mavsdk::ParamServer *mMavsdkParamServer;
    std::unordered_map<std::string, std::pair<std::function<void(float)>, std::function<float(void)>>> mParameterToClassMapping;
};

#endif // PARAMETERSERVER_H
