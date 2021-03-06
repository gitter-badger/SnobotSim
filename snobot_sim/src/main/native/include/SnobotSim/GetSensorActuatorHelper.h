/*
 * GetSensorActuatorHelper.h
 *
 *  Created on: May 19, 2017
 *      Author: preiniger
 */

#ifndef SRC_GETSENSORACTUATORHELPER_H_
#define SRC_GETSENSORACTUATORHELPER_H_

#include "SnobotSim/SensorActuatorRegistry.h"

namespace GetSensorActuatorHelper
{
    EXPORT_ std::shared_ptr<SpeedControllerWrapper> GetSpeedControllerWrapper(int aHandle, bool aLogOnMissing=true);
    EXPORT_ std::shared_ptr<IGyroWrapper> GetIGyroWrapper(int aHandle, bool aLogOnMissing=true);
    EXPORT_ std::shared_ptr<DigitalSourceWrapper> GetDigitalSourceWrapper(int aHandle, bool aLogOnMissing=true);
    EXPORT_ std::shared_ptr<AnalogSourceWrapper> GetAnalogSourceWrapper(int aHandle, bool aLogOnMissing=true);
    EXPORT_ std::shared_ptr<RelayWrapper> GetRelayWrapper(int aHandle, bool aLogOnMissing=true);
    EXPORT_ std::shared_ptr<SolenoidWrapper> GetSolenoidWrapper(int aHandle, bool aLogOnMissing=true);
    EXPORT_ std::shared_ptr<EncoderWrapper> GetEncoderWrapper(int aHandle, bool aLogOnMissing=true);
    EXPORT_ std::shared_ptr<ISpiWrapper> GetISpiWrapper(int aHandle, bool aLogOnMissing=false);
    EXPORT_ std::shared_ptr<II2CWrapper> GetII2CWrapper(int aHandle, bool aLogOnMissing=false);
}

#endif /* SRC_GETSENSORACTUATORHELPER_H_ */
