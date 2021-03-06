/*
 * SensorActuatorRegistry.cpp
 *
 *  Created on: May 3, 2017
 *      Author: PJ
 */

#include "SnobotSim/SensorActuatorRegistry.h"
#include "SnobotSim/SimulatorComponents/CompressorWrapper.h"
#include "HAL/handles/HandlesInternal.h"
#include "SnobotSim/SimulatorComponents/Ctre/CanManager.h"

SensorActuatorRegistry* SensorActuatorRegistry::sInstance =
        new SensorActuatorRegistry();


SensorActuatorRegistry& SensorActuatorRegistry::Get()
{
    return *sInstance;
}
    
SensorActuatorRegistry::SensorActuatorRegistry() :
        mCompressor(new CompressorWrapper)
{

}

SensorActuatorRegistry::~SensorActuatorRegistry()
{
    Reset();
}


void SensorActuatorRegistry::Reset()
{
    hal::HandleBase::ResetGlobalHandles();

    mSpeedControllerWrapperMap.clear();
    mRelayWrapperMap.clear();
    mDigitalSourceWrapperMap.clear();
    mAnalogSourceWrapperMap.clear();
    mSolenoidWrapperMap.clear();
    mEncoderWrapperMap.clear();
    mIGyroWrapperMap.clear();
    mIAccelerometerWrapperMap.clear();
    mISpiWrapperMap.clear();
    mII2CWrapperMap.clear();

    mSimulatorComponents.clear();

    mCanManager = std::shared_ptr<CanManager>(new CanManager);
}

std::shared_ptr<CompressorWrapper> SensorActuatorRegistry::GetCompressorWrapper()
{
    return mCompressor;
}

void SensorActuatorRegistry::AddSimulatorComponent(const std::shared_ptr<ISimulatorUpdater>& aSimulatorComponent)
{
    mSimulatorComponents.push_back(aSimulatorComponent);
}


std::vector<std::shared_ptr<ISimulatorUpdater>>& SensorActuatorRegistry::GetSimulatorComponents()
{
    return mSimulatorComponents;
}


#define ACTUATOR_GETTERS(ItemType, aOverwriteOnConflict)                                                        \
    bool SensorActuatorRegistry::Register(int aPort,  const std::shared_ptr<ItemType>& aActuator)               \
    {                                                                                                           \
        return RegisterItem(aPort, aActuator, m##ItemType##Map, #ItemType, aOverwriteOnConflict);               \
    }                                                                                                           \
    std::shared_ptr<ItemType> SensorActuatorRegistry::Get##ItemType(int aPort, bool aLogError)                  \
    {                                                                                                           \
        return GetItem(aPort, m##ItemType##Map, #ItemType, aLogError);                                          \
    }                                                                                                           \
    const std::map<int, std::shared_ptr<ItemType>>& SensorActuatorRegistry::Get##ItemType##Map() const          \
    {                                                                                                           \
        return m##ItemType##Map;                                                                                \
    }                                                                                                           \
    std::map<int, std::shared_ptr<ItemType>>& SensorActuatorRegistry::Get##ItemType##Map()                      \
    {                                                                                                           \
        return m##ItemType##Map;                                                                                \
    }                                                                                                           \

ACTUATOR_GETTERS(SpeedControllerWrapper, false)
ACTUATOR_GETTERS(RelayWrapper, false)
ACTUATOR_GETTERS(DigitalSourceWrapper, false)
ACTUATOR_GETTERS(AnalogSourceWrapper, false)
ACTUATOR_GETTERS(SolenoidWrapper, false)
ACTUATOR_GETTERS(EncoderWrapper, false)
ACTUATOR_GETTERS(IAccelerometerWrapper, false)
ACTUATOR_GETTERS(IGyroWrapper, false)
ACTUATOR_GETTERS(ISpiWrapper, true)
ACTUATOR_GETTERS(II2CWrapper, true)
