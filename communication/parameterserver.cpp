/*
 *     Copyright 2023 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "parameterserver.h"

void ParameterServer::updateParameter(std::string parameterName, float parameterValue)
{
    auto setParameterFunction = mParameterToClassMapping.find(parameterName)->second.first;
    setParameterFunction(parameterValue);
}

void ParameterServer::parameterToClassMapping(std::string parameterName, std::function<void(float)> setClassParameterFunction, std::function<float(void)> getClassParameterFunction)
{
    mParameterToClassMapping.insert_or_assign(parameterName, std::make_pair(setClassParameterFunction, getClassParameterFunction));
};
