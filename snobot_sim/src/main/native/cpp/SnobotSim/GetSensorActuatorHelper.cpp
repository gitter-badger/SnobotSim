/*
 * GetSensorActuatorHelper.cpp
 *
 *  Created on: May 19, 2017
 *      Author: preiniger
 */

#include "SnobotSim/GetSensorActuatorHelper.h"
#include "SnobotSim/PortUnwrapper.h"

#define FIND_MODULE_FUNC(WrapperType) \
std::shared_ptr<WrapperType> Get##WrapperType(int aHandle, bool aLogOnMissing)                                                            \
{                                                                                                                                         \
    std::shared_ptr<WrapperType> wrapper = SensorActuatorRegistry::Get().Get##WrapperType(aHandle, false);                                \
    if(!wrapper)                                                                                                                          \
    {                                                                                                                                     \
        int packedPort = WrapPort(aHandle);                                                                                               \
        wrapper = SensorActuatorRegistry::Get().Get##WrapperType(packedPort, false);                                                      \
        if(!wrapper && aLogOnMissing)                                                                                                     \
        {                                                                                                                                 \
            SNOBOT_LOG(SnobotLogging::CRITICAL, "Could not find " << #WrapperType << " for handles " << aHandle << " or " << packedPort); \
        }                                                                                                                                 \
    }                                                                                                                                     \
    return wrapper;                                                                                                                       \
}

namespace GetSensorActuatorHelper
{
    FIND_MODULE_FUNC(SpeedControllerWrapper)
    FIND_MODULE_FUNC(IGyroWrapper)
    FIND_MODULE_FUNC(DigitalSourceWrapper)
    FIND_MODULE_FUNC(AnalogSourceWrapper)
    FIND_MODULE_FUNC(RelayWrapper)
    FIND_MODULE_FUNC(SolenoidWrapper)
    FIND_MODULE_FUNC(EncoderWrapper)
    FIND_MODULE_FUNC(ISpiWrapper)
    FIND_MODULE_FUNC(II2CWrapper)
}

