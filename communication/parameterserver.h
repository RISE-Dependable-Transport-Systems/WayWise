/*
 *     Copyright 2023 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#ifndef PARAMETERSERVER_H
#define PARAMETERSERVER_H

#include <QObject>
#include <mutex>

class ParameterServer : public QObject
{
    Q_OBJECT
public:
    struct IntParameter {
        std::string name{};
        int32_t value{};
    };
    struct FloatParameter {
        std::string name{};
        float value{};
    };
    struct CustomParameter {
        std::string name{};
        std::string value{};
    };
    struct AllParameters {
        std::vector<IntParameter>
            intParameters{};
        std::vector<FloatParameter> floatParameters{};
        std::vector<CustomParameter> customParameters{};
    };

    static void initialize();
    static ParameterServer* getInstance();
    bool updateParameter(std::string parameterName, float parameterValue);
    virtual void provideParameter(std::string parameterName, std::function<void(float)> setClassParameterFunction, std::function<float(void)> getClassParameterFunction);
    virtual void saveParametersToXmlFile(QString filename);
    AllParameters getAllParameters();

protected:
    ParameterServer();
    virtual ~ParameterServer(){};
    static ParameterServer *mInstancePtr;
    std::mutex mMutex;
    std::unordered_map<std::string, std::pair<std::function<void(float)>, std::function<float(void)>>> mParameterToClassMapping;
};

#endif // PARAMETERSERVER_H
