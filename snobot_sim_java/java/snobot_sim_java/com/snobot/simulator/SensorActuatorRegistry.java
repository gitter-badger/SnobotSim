package com.snobot.simulator;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

import com.snobot.simulator.module_wrapper.AnalogWrapper;
import com.snobot.simulator.module_wrapper.DigitalSourceWrapper;
import com.snobot.simulator.module_wrapper.EncoderWrapper;
import com.snobot.simulator.module_wrapper.PwmWrapper;
import com.snobot.simulator.module_wrapper.RelayWrapper;
import com.snobot.simulator.module_wrapper.SolenoidWrapper;
import com.snobot.simulator.simulator_components.II2CWrapper;
import com.snobot.simulator.simulator_components.ISimulatorUpdater;
import com.snobot.simulator.simulator_components.ISpiWrapper;
import com.snobot.simulator.simulator_components.accelerometer.IAccelerometerWrapper;
import com.snobot.simulator.simulator_components.gyro.IGyroWrapper;

public final class SensorActuatorRegistry
{
    private static SensorActuatorRegistry mInstance = new SensorActuatorRegistry();

    private final Map<Integer, PwmWrapper> mSpeedControllerMap = new HashMap<>();
    private final Map<Integer, RelayWrapper> mRelayWrapperMap = new HashMap<>();
    private final Map<Integer, DigitalSourceWrapper> mDigitalSourceWrapperMap = new HashMap<>();
    private final Map<Integer, AnalogWrapper> mAnalogSourceWrapperMap = new HashMap<>();
    private final Map<Integer, SolenoidWrapper> mSolenoidWrapperMap = new HashMap<>();
    private final Map<Integer, EncoderWrapper> mEncoderWrapperMap = new HashMap<>();

    private final Map<Integer, IGyroWrapper> mGyroWrapperMap = new HashMap<>();
    private final Map<Integer, IAccelerometerWrapper> mAccelerometerWrapperMap = new HashMap<>();
    private final Map<Integer, II2CWrapper> mI2CWrapperMap = new HashMap<>();
    private final Map<Integer, ISpiWrapper> mSpiWrapperMap = new HashMap<>();

    private final Collection<ISimulatorUpdater> mSimulatorComponents = new ArrayList<>();

    private SensorActuatorRegistry()
    {

    }

    public static SensorActuatorRegistry get()
    {
        return mInstance;
    }

    public <ItemType> boolean registerItem(ItemType aItem, int aPort, Map<Integer, ItemType> aMap, String aMessage)
    {
        aMap.put(aPort, aItem);
        return true;
    }

    public boolean register(AnalogWrapper aActuator, int aPort)
    {
        return registerItem(aActuator, aPort, mAnalogSourceWrapperMap, "Analog");
    }

    public boolean register(PwmWrapper aActuator, int aPort)
    {
        return registerItem(aActuator, aPort, mSpeedControllerMap, "Speed Controller");
    }

    public boolean register(DigitalSourceWrapper aSensor, int aPort)
    {
        return registerItem(aSensor, aPort, mDigitalSourceWrapperMap, "Digital IO");
    }

    public boolean register(SolenoidWrapper aActuator, int aPort)
    {
        return registerItem(aActuator, aPort, mSolenoidWrapperMap, "Solenoid");
    }

    public boolean register(RelayWrapper aActuator, int aPort)
    {
        return registerItem(aActuator, aPort, mRelayWrapperMap, "Relay");
    }

    public boolean register(EncoderWrapper aEncoder, Integer aPort)
    {
        return registerItem(aEncoder, aPort, mEncoderWrapperMap, "Encoder");
    }

    public boolean register(IAccelerometerWrapper aSensor, Integer aPort)
    {
        return registerItem(aSensor, aPort, mAccelerometerWrapperMap, "Accelerometer");
    }

    public boolean register(IGyroWrapper aSensor, Integer aPort)
    {
        return registerItem(aSensor, aPort, mGyroWrapperMap, "Gyro");
    }

    public boolean register(II2CWrapper aSensor, Integer aPort)
    {
        return registerItem(aSensor, aPort, mI2CWrapperMap, "I2C");
    }

    public boolean register(ISpiWrapper aSensor, Integer aPort)
    {
        return registerItem(aSensor, aPort, mSpiWrapperMap, "SPI");
    }

    public boolean register(ISimulatorUpdater aUpdater)
    {
        mSimulatorComponents.add(aUpdater);
        return true;
    }

    public Map<Integer, PwmWrapper> getSpeedControllers()
    {
        return mSpeedControllerMap;
    }

    public Map<Integer, SolenoidWrapper> getSolenoids()
    {
        return mSolenoidWrapperMap;
    }

    public Map<Integer, DigitalSourceWrapper> getDigitalSources()
    {
        return mDigitalSourceWrapperMap;
    }

    public Map<Integer, RelayWrapper> getRelays()
    {
        return mRelayWrapperMap;
    }

    public Map<Integer, AnalogWrapper> getAnalog()
    {
        return mAnalogSourceWrapperMap;
    }

    public Map<Integer, EncoderWrapper> getEncoders()
    {
        return mEncoderWrapperMap;
    }

    public Map<Integer, IAccelerometerWrapper> getAccelerometers()
    {
        return mAccelerometerWrapperMap;
    }

    public Map<Integer, IGyroWrapper> getGyros()
    {
        return mGyroWrapperMap;
    }

    public Map<Integer, II2CWrapper> getI2CWrapperss()
    {
        return mI2CWrapperMap;
    }

    public Map<Integer, ISpiWrapper> getSpiWrappers()
    {
        return mSpiWrapperMap;
    }

    public Collection<ISimulatorUpdater> getSimulatorComponents()
    {
        return mSimulatorComponents;
    }

    public void reset()
    {
        for (ISpiWrapper wrapper : mSpiWrapperMap.values())
        {
            if (wrapper != null)
            {
                wrapper.shutdown();
            }
        }
        for (II2CWrapper wrapper : mI2CWrapperMap.values())
        {
            if (wrapper != null)
            {
                wrapper.shutdown();
            }
        }

        mSpeedControllerMap.clear();
        mRelayWrapperMap.clear();
        mDigitalSourceWrapperMap.clear();
        mAnalogSourceWrapperMap.clear();
        mSolenoidWrapperMap.clear();
        mEncoderWrapperMap.clear();

        mGyroWrapperMap.clear();
        mAccelerometerWrapperMap.clear();
        mI2CWrapperMap.clear();
        mSpiWrapperMap.clear();

        mSimulatorComponents.clear();
    }
}
