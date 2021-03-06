/*
 * SensorActuatorRegistry.h
 *
 *  Created on: May 3, 2017
 *      Author: PJ
 */

#ifndef INCLUDE_SNOBOTSIM_SENSORACTUATORREGISTRY_H_
#define INCLUDE_SNOBOTSIM_SENSORACTUATORREGISTRY_H_

#include <map>
#include <memory>
#include <vector>
#include "SnobotSim/ExportHelper.h"
#include "SnobotSim/SimulatorComponents/ISimulatorUpdater.h"
#include "SnobotSim/Logging/SnobotLogger.h"
#include "SnobotSim/StackHelper/StackTraceHelper.h"

class SpeedControllerWrapper;
class RelayWrapper;
class DigitalSourceWrapper;
class AnalogSourceWrapper;
class SolenoidWrapper;
class EncoderWrapper;
class IGyroWrapper;
class IAccelerometerWrapper;
class ISpiWrapper;
class II2CWrapper;
class CompressorWrapper;
class CanManager;

#define ACTUATOR_GETTERS(ItemType)                                              \
    bool Register(int aPort, const std::shared_ptr<ItemType>& aActuator);       \
    std::shared_ptr<ItemType> Get##ItemType(int aPort, bool aLogError = true);  \
    const std::map<int, std::shared_ptr<ItemType>>& Get##ItemType##Map() const; \
    std::map<int, std::shared_ptr<ItemType>>& Get##ItemType##Map();


class EXPORT_ SensorActuatorRegistry
{
private:
    SensorActuatorRegistry();
    static SensorActuatorRegistry* sInstance;

public:
    virtual ~SensorActuatorRegistry();

    static SensorActuatorRegistry& Get();

    void AddSimulatorComponent(const std::shared_ptr<ISimulatorUpdater>& aSimulatorComponent);

    void Reset();

    std::vector<std::shared_ptr<ISimulatorUpdater>>& GetSimulatorComponents();

    std::shared_ptr<CompressorWrapper> GetCompressorWrapper();

    ACTUATOR_GETTERS(SpeedControllerWrapper)
    ACTUATOR_GETTERS(RelayWrapper)
    ACTUATOR_GETTERS(DigitalSourceWrapper)
    ACTUATOR_GETTERS(AnalogSourceWrapper)
    ACTUATOR_GETTERS(SolenoidWrapper)
    ACTUATOR_GETTERS(EncoderWrapper)
    ACTUATOR_GETTERS(IAccelerometerWrapper)
    ACTUATOR_GETTERS(IGyroWrapper)
    ACTUATOR_GETTERS(ISpiWrapper)
    ACTUATOR_GETTERS(II2CWrapper)

protected:

    template<typename ItemType>
    bool RegisterItem(int aPort, std::shared_ptr<ItemType> aItem,
            std::map<int, std::shared_ptr<ItemType>>& aMap, const std::string& aType,
            bool aOverwriteOnConflict)
    {
        if (aMap.find(aPort) != aMap.end())
        {
            if(aOverwriteOnConflict)
            {
                SNOBOT_LOG(SnobotLogging::INFO, "Overwriting registration of " << aType << " on port " << aPort);
            }
            else
            {
                SNOBOT_LOG(SnobotLogging::WARN, "Type " << aType << " already has registered item on port " << aPort << ".");
                return false;
            }
        }

        aMap[aPort] = aItem;

        SNOBOT_LOG(SnobotLogging::DEBUG, "Registered " << aType << " on port " << aPort << ".  The map has " << aMap.size() << " elements.")

        return true;
    }

    template<typename ItemType>
    std::shared_ptr<ItemType> GetItem(int aPort,
            const std::map<int, std::shared_ptr<ItemType>>& aMap, const std::string& aType, bool logError)
    {
        typename std::map<int, std::shared_ptr<ItemType>>::const_iterator iter =
                aMap.find(aPort);
        if (iter == aMap.end())
        {
            if (logError)
            {
                SNOBOT_LOG(SnobotLogging::CRITICAL, "Unregistered " << aType << " on port " << aPort << ".  Map has " << aMap.size() << " elements.")
                StackTraceHelper::PrintStackTracker();
            }
            return std::shared_ptr<ItemType>();
        }

        return iter->second;
    }

    std::map<int, std::shared_ptr<SpeedControllerWrapper>> mSpeedControllerWrapperMap;
    std::map<int, std::shared_ptr<RelayWrapper>> mRelayWrapperMap;
    std::map<int, std::shared_ptr<DigitalSourceWrapper>> mDigitalSourceWrapperMap;
    std::map<int, std::shared_ptr<AnalogSourceWrapper>> mAnalogSourceWrapperMap;
    std::map<int, std::shared_ptr<SolenoidWrapper>> mSolenoidWrapperMap;
    std::map<int, std::shared_ptr<EncoderWrapper>> mEncoderWrapperMap;
    std::map<int, std::shared_ptr<IGyroWrapper>> mIGyroWrapperMap;
    std::map<int, std::shared_ptr<IAccelerometerWrapper>> mIAccelerometerWrapperMap;
    std::map<int, std::shared_ptr<ISpiWrapper>> mISpiWrapperMap;
    std::map<int, std::shared_ptr<II2CWrapper>> mII2CWrapperMap;

    std::shared_ptr<CompressorWrapper> mCompressor;
    std::shared_ptr<CanManager> mCanManager;

    std::vector<std::shared_ptr<ISimulatorUpdater>> mSimulatorComponents;
};

#undef ACTUATOR_GETTERS

#endif /* INCLUDE_SNOBOTSIM_SENSORACTUATORREGISTRY_H_ */
