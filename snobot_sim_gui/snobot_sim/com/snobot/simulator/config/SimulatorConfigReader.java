package com.snobot.simulator.config;

import java.io.File;
import java.io.FileReader;
import java.util.List;
import java.util.Map.Entry;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.yaml.snakeyaml.Yaml;

import com.snobot.simulator.motor_sim.DcMotorModelConfig;
import com.snobot.simulator.motor_sim.GravityLoadMotorSimulationConfig;
import com.snobot.simulator.motor_sim.IMotorSimulatorConfig;
import com.snobot.simulator.motor_sim.RotationalLoadMotorSimulationConfig;
import com.snobot.simulator.motor_sim.SimpleMotorSimulationConfig;
import com.snobot.simulator.motor_sim.StaticLoadMotorSimulationConfig;
import com.snobot.simulator.simulator_components.TankDriveGyroSimulator;
import com.snobot.simulator.wrapper_accessors.DataAccessorFactory;
import com.snobot.simulator.wrapper_accessors.EncoderWrapperAccessor;
import com.snobot.simulator.wrapper_accessors.IBasicSensorActuatorWrapperAccessor;
import com.snobot.simulator.wrapper_accessors.SimulatorDataAccessor;
import com.snobot.simulator.wrapper_accessors.SpeedControllerWrapperAccessor;

/**
 * Class to read the configuration file and set up the simulator components
 * 
 * @author PJ
 *
 */
public class SimulatorConfigReader
{
    private static final Logger sLOGGER = Logger.getLogger(SimulatorConfigReader.class);

    private SimulatorConfig mConfig;

    public SimulatorConfigReader()
    {
        mConfig = null;
    }

    /**
     * Loads the given config file. Calling this will save off the config, but
     * not actually make any of the {@code setName()} or
     * {@code hookUpSpeedController()} calls, as the robot may not be set up
     * yet. It will however setup the SPI and I2C factories
     * 
     * @param aConfigFile
     *            The config file to load
     * @return True if the config was loaded succesfully
     */
    public boolean loadConfig(String aConfigFile)
    {
        if (aConfigFile == null)
        {
            sLOGGER.log(Level.WARN, "Config file not set, won't hook anything up");
            return true;
        }

        boolean success = false;

        try
        {
            File file = new File(aConfigFile);
            sLOGGER.log(Level.INFO, "Loading " + file.getAbsolutePath());
            Yaml yaml = new Yaml();
            mConfig = (SimulatorConfig) yaml.load(new FileReader(file));

            if (mConfig != null)
            {
                for (Entry<Integer, String> pair : mConfig.getmDefaultI2CWrappers().entrySet())
                {
                    DataAccessorFactory.getInstance().getSimulatorDataAccessor().setDefaultI2CSimulator(pair.getKey(), pair.getValue());
                }
                for (Entry<Integer, String> pair : mConfig.getmDefaultSpiWrappers().entrySet())
                {
                    DataAccessorFactory.getInstance().getSimulatorDataAccessor().setDefaultSpiSimulator(pair.getKey(), pair.getValue());
                }
            }

            success = true;
        }
        catch (Exception e)
        {
            sLOGGER.log(Level.ERROR, e);
        }

        return success;
    }

    /**
     * Returns the config loaded during the {@link #loadConfig} function
     * 
     * @return The config
     */
    public SimulatorConfig getConfig()
    {
        return mConfig;
    }

    public void setupSimulatorComponents()
    {
        if (mConfig != null)
        {
            createBasicSimulatorComponents(DataAccessorFactory.getInstance().getDigitalAccessor(), mConfig.getmDigitalIO());
            createBasicSimulatorComponents(DataAccessorFactory.getInstance().getAnalogAccessor(), mConfig.getmAnalogIO());
            createBasicSimulatorComponents(DataAccessorFactory.getInstance().getRelayAccessor(), mConfig.getmRelays());
            createBasicSimulatorComponents(DataAccessorFactory.getInstance().getSolenoidAccessor(), mConfig.getmSolenoids());
            createEncoders(DataAccessorFactory.getInstance().getEncoderAccessor(), mConfig.getmEncoders());
            createPwms(DataAccessorFactory.getInstance().getSpeedControllerAccessor(), mConfig.getmPwm());

            for (Object obj : mConfig.getmSimulatorComponents())
            {
                setupSimulatorComponents(obj);
            }
        }
    }

    protected void setupSimulatorComponents(Object aConfig)
    {
        if (aConfig instanceof TankDriveGyroSimulator.TankDriveConfig)
        {
            TankDriveGyroSimulator.TankDriveConfig config = (TankDriveGyroSimulator.TankDriveConfig) aConfig;
            DataAccessorFactory.getInstance().getSimulatorDataAccessor().connectTankDriveSimulator(config.getmLeftEncoderHandle(),
                    config.getmRightEncoderHandle(), config.getmGyroHandle(), config.getmTurnKp());
        }
    }

    protected void createBasicSimulatorComponents(IBasicSensorActuatorWrapperAccessor aAccessor, List<? extends BasicModuleConfig> aInputList)
    {
        for (BasicModuleConfig config : aInputList)
        {
            int handle = config.getmHandle();
            if (handle != -1 && config.getmName() != null)
            {
                aAccessor.setName(handle, config.getmName());
            }
        }
    }

    protected void createEncoders(EncoderWrapperAccessor aAccessor, List<EncoderConfig> aInputList)
    {
        for (EncoderConfig config : aInputList)
        {
            int handle = config.getmHandle();
            if (handle != -1)
            {
                if (config.getmName() != null)
                {
                    aAccessor.setName(handle, config.getmName());
                }
                if (config.getmConnectedSpeedControllerHandle() != -1)
                {
                    aAccessor.connectSpeedController(handle, config.getmConnectedSpeedControllerHandle());
                }
            }
        }
    }

    protected void createPwms(SpeedControllerWrapperAccessor aAccessor, List<PwmConfig> aInputList)
    {
        for (PwmConfig config : aInputList)
        {
            int handle = config.getmHandle();
            if (handle != -1)
            {
                if (config.getmName() != null)
                {
                    aAccessor.setName(handle, config.getmName());
                }

                setupMotorSimulator(config, handle);
            }
        }
    }

    protected void setupMotorSimulator(PwmConfig aPwmConfig, int aPwmHandle)
    {
        SimulatorDataAccessor simulatorAccessor = DataAccessorFactory.getInstance().getSimulatorDataAccessor();
        IMotorSimulatorConfig baseMotorConfig = aPwmConfig.getmMotorSimConfig();
        DcMotorModelConfig.FactoryParams motorConfigFactoryParams = aPwmConfig.getmMotorModelConfig();
        DcMotorModelConfig motorConfig;
        if (motorConfigFactoryParams != null)
        {
            motorConfig = simulatorAccessor.createMotor(motorConfigFactoryParams.mMotorType, motorConfigFactoryParams.mNumMotors,
                    motorConfigFactoryParams.mGearReduction, motorConfigFactoryParams.mGearboxEfficiency, motorConfigFactoryParams.mInverted,
                    motorConfigFactoryParams.mHasBrake);
        }
        else
        {
            motorConfig = new DcMotorModelConfig(motorConfigFactoryParams, null);
        }

        if (baseMotorConfig != null)
        {
            if (baseMotorConfig instanceof SimpleMotorSimulationConfig)
            {
                simulatorAccessor.setSpeedControllerModel_Simple(aPwmHandle, (SimpleMotorSimulationConfig) baseMotorConfig);
            }
            else if (baseMotorConfig instanceof StaticLoadMotorSimulationConfig)
            {
                simulatorAccessor.setSpeedControllerModel_Static(aPwmHandle, motorConfig, (StaticLoadMotorSimulationConfig) baseMotorConfig);
            }
            else if (baseMotorConfig instanceof GravityLoadMotorSimulationConfig)
            {
                simulatorAccessor.setSpeedControllerModel_Gravitational(aPwmHandle, motorConfig, (GravityLoadMotorSimulationConfig) baseMotorConfig);
            }
            else if (baseMotorConfig instanceof RotationalLoadMotorSimulationConfig)
            {
                simulatorAccessor.setSpeedControllerModel_Rotational(aPwmHandle, motorConfig, (RotationalLoadMotorSimulationConfig) baseMotorConfig);
            }
        }
    }
}
