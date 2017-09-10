package com.snobot.simulator.wrapper_accessors.jni;

import com.snobot.simulator.jni.JoystickJni;
import com.snobot.simulator.jni.MotorConfigFactoryJni;
import com.snobot.simulator.jni.RobotStateSingletonJni;
import com.snobot.simulator.jni.SimulationConnectorJni;
import com.snobot.simulator.jni.SnobotSimulatorJni;
import com.snobot.simulator.motor_sim.DcMotorModelConfig;
import com.snobot.simulator.motor_sim.GravityLoadMotorSimulationConfig;
import com.snobot.simulator.motor_sim.RotationalLoadMotorSimulationConfig;
import com.snobot.simulator.motor_sim.SimpleMotorSimulationConfig;
import com.snobot.simulator.motor_sim.StaticLoadMotorSimulationConfig;
import com.snobot.simulator.wrapper_accessors.SimulatorDataAccessor;

public class JniSimulatorDataAccessor implements SimulatorDataAccessor
{

    @Override
    public void setLogLevel(SnobotLogLevel logLevel)
    {
        SnobotSimulatorJni.initializeLogging(logLevel.ordinal());
    }

    @Override
    public String getNativeBuildVersion()
    {
        throw new UnsupportedOperationException();
    }

    @Override
    public void reset()
    {
        SnobotSimulatorJni.reset();
    }

    @Override
    public boolean connectTankDriveSimulator(int leftEncHandle, int rightEncHandle, int aGyroHandle, double turnKp)
    {
        return SimulationConnectorJni.connectTankDriveSimulator(leftEncHandle, rightEncHandle, aGyroHandle, turnKp);
    }

    @Override
    public DcMotorModelConfig createMotor(String motorType, int numMotors, double gearReduction, double efficiency)
    {
        return MotorConfigFactoryJni.createMotor(motorType, numMotors, gearReduction, efficiency);
    }

    @Override
    public DcMotorModelConfig createMotor(String motorType)
    {
        return MotorConfigFactoryJni.createMotor(motorType);
    }

    @Override
    public boolean setSpeedControllerModel_Simple(int aScHandle, SimpleMotorSimulationConfig aConfig)
    {
        return SimulationConnectorJni.setSpeedControllerModel_Simple(aScHandle, aConfig.mMaxSpeed);
    }

    @Override
    public boolean setSpeedControllerModel_Static(int aScHandle, DcMotorModelConfig aMotorConfig, StaticLoadMotorSimulationConfig aConfig)
    {
        return SimulationConnectorJni.setSpeedControllerModel_Static(aScHandle, aMotorConfig, aConfig.mLoad, aConfig.mConversionFactor);
    }

    @Override
    public boolean setSpeedControllerModel_Gravitational(int aScHandle, DcMotorModelConfig aMotorConfig, GravityLoadMotorSimulationConfig aConfig)
    {
        return SimulationConnectorJni.setSpeedControllerModel_Gravitational(aScHandle, aMotorConfig, aConfig.mLoad);
    }

    @Override
    public boolean setSpeedControllerModel_Rotational(int aScHandle, DcMotorModelConfig aMotorConfig, RotationalLoadMotorSimulationConfig aConfig)
    {
        return SimulationConnectorJni.setSpeedControllerModel_Rotational(aScHandle, aMotorConfig, aConfig.mArmCenterOfMass, aConfig.mArmMass,
                aConfig.mConstantAssistTorque, aConfig.mOverCenterAssistTorque);
    }

    @Override
    public void setDisabled(boolean aDisabled)
    {
        RobotStateSingletonJni.setDisabled(aDisabled);
    }

    @Override
    public void setAutonomous(boolean aAuton)
    {
        RobotStateSingletonJni.setAutonomous(aAuton);
    }

    @Override
    public double getMatchTime()
    {
        return RobotStateSingletonJni.getMatchTime();
    }

    @Override
    public void waitForProgramToStart()
    {
        RobotStateSingletonJni.waitForProgramToStart();
    }

    @Override
    public void updateSimulatorComponents(double aUpdatePeriod)
    {
        SimulationConnectorJni.updateLoop();
    }

    @Override
    public void waitForNextUpdateLoop(double aUpdatePeriod)
    {
        RobotStateSingletonJni.waitForNextUpdateLoop();
    }

    @Override
    public void setJoystickInformation(int i, float[] axisValues, short[] povValues, int buttonCount, int buttonMask)
    {
        JoystickJni.setJoystickInformation(i, axisValues, povValues, buttonCount, buttonMask);
    }

}
