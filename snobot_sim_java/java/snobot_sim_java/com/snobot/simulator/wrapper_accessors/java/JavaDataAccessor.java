package com.snobot.simulator.wrapper_accessors.java;

import com.snobot.simulator.LogConfigurator;
import com.snobot.simulator.jni.RegisterCallbacksJni;
import com.snobot.simulator.wrapper_accessors.AccelerometerWrapperAccessor;
import com.snobot.simulator.wrapper_accessors.AnalogSourceWrapperAccessor;
import com.snobot.simulator.wrapper_accessors.DigitalSourceWrapperAccessor;
import com.snobot.simulator.wrapper_accessors.EncoderWrapperAccessor;
import com.snobot.simulator.wrapper_accessors.GyroWrapperAccessor;
import com.snobot.simulator.wrapper_accessors.IDataAccessor;
import com.snobot.simulator.wrapper_accessors.RelayWrapperAccessor;
import com.snobot.simulator.wrapper_accessors.SimulatorDataAccessor;
import com.snobot.simulator.wrapper_accessors.SolenoidWrapperAccessor;
import com.snobot.simulator.wrapper_accessors.SpeedControllerWrapperAccessor;

public class JavaDataAccessor implements IDataAccessor
{
    private final AccelerometerWrapperAccessor mAccelerometer;
    private final GyroWrapperAccessor mGyro;
    private final AnalogSourceWrapperAccessor mAnalog;
    private final DigitalSourceWrapperAccessor mDigital;
    private final EncoderWrapperAccessor mEncoder;
    private final RelayWrapperAccessor mRelay;
    private final SolenoidWrapperAccessor mSolenoid;
    private final SpeedControllerWrapperAccessor mPwm;
    private final SimulatorDataAccessor mSimulator;

    public JavaDataAccessor()
    {
        LogConfigurator.loadLog4jConfig();
        RegisterCallbacksJni.registerAllCallbacks();

        mAccelerometer = new JavaAccelerometerWrapperAccessor();
        mGyro = new JavaGyroWrapperAccessor();
        mAnalog = new JavaAnalogSourceWrapperAccessor();
        mDigital = new JavaDigitalSourceWrapperAccessor();
        mEncoder = new JavaEncoderWrapperAccessor();
        mRelay = new JavaRelayWrapperAccessor();
        mSolenoid = new JavaSolenoidWrapperAccessor();
        mPwm = new JavaSpeedControllerWrapperAccessor();
        mSimulator = new JavaSimulatorDataAccessor();
    }

    @Override
    public String getAccessorType()
    {
        return "Java";
    }

    @Override
    public AccelerometerWrapperAccessor getAccelerometerAccessor()
    {
        return mAccelerometer;
    }

    @Override
    public GyroWrapperAccessor getGyroAccessor()
    {
        return mGyro;
    }

    @Override
    public AnalogSourceWrapperAccessor getAnalogAccessor()
    {
        return mAnalog;
    }

    @Override
    public DigitalSourceWrapperAccessor getDigitalAccessor()
    {
        return mDigital;
    }

    @Override
    public EncoderWrapperAccessor getEncoderAccessor()
    {
        return mEncoder;
    }

    @Override
    public RelayWrapperAccessor getRelayAccessor()
    {
        return mRelay;
    }

    @Override
    public SolenoidWrapperAccessor getSolenoidAccessor()
    {
        return mSolenoid;
    }

    @Override
    public SpeedControllerWrapperAccessor getSpeedControllerAccessor()
    {
        return mPwm;
    }

    @Override
    public SimulatorDataAccessor getSimulatorDataAccessor()
    {
        return mSimulator;
    }
}
