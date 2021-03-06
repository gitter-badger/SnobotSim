package com.snobot.simulator.config;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.yaml.snakeyaml.DumperOptions;
import org.yaml.snakeyaml.Yaml;

import com.snobot.simulator.motor_sim.DcMotorModelConfig;
import com.snobot.simulator.motor_sim.IMotorSimulatorConfig;
import com.snobot.simulator.wrapper_accessors.DataAccessorFactory;
import com.snobot.simulator.wrapper_accessors.EncoderWrapperAccessor;
import com.snobot.simulator.wrapper_accessors.IBasicSensorActuatorWrapperAccessor;
import com.snobot.simulator.wrapper_accessors.SimulatorDataAccessor;
import com.snobot.simulator.wrapper_accessors.SpeedControllerWrapperAccessor;
import com.snobot.simulator.wrapper_accessors.SpeedControllerWrapperAccessor.MotorSimType;

/**
 * This class writes the configuration information to a config file
 * 
 * @author PJ
 *
 */
public class SimulatorConfigWriter
{
    private static final Logger sLOGGER = Logger.getLogger(SimulatorConfigWriter.class);

    /**
     * Looks at all of the loaded simulator components and writes the config to
     * a file
     * 
     * @param aOutFile
     *            The output configuration file path
     * @return True if the write was successful
     */
    public boolean writeConfig(String aOutFile)
    {
        boolean success = false;

        try
        {
            File file = new File(aOutFile);
            sLOGGER.log(Level.INFO, "Writing to " + file.getAbsolutePath());

            Object output = dumpConfig();

            DumperOptions options = new DumperOptions();
            options.setDefaultFlowStyle(DumperOptions.FlowStyle.BLOCK);

            Yaml yaml = new Yaml(options);
            yaml.dump(output, new FileWriter(file));

            success = true;
        }
        catch (IOException e)
        {
            sLOGGER.log(Level.ERROR, e);
        }

        return success;
    }

    protected SimulatorConfig dumpConfig()
    {
        // Map<String, Object> output = new LinkedHashMap<>();

        SimulatorConfig output = new SimulatorConfig();

        dumpBasicConfig(DataAccessorFactory.getInstance().getAccelerometerAccessor(), output.getmAccelerometers());
        dumpBasicConfig(DataAccessorFactory.getInstance().getAnalogAccessor(), output.getmAnalogIO());
        dumpBasicConfig(DataAccessorFactory.getInstance().getDigitalAccessor(), output.getmDigitalIO());
        dumpBasicConfig(DataAccessorFactory.getInstance().getGyroAccessor(), output.getmGyros());
        dumpBasicConfig(DataAccessorFactory.getInstance().getRelayAccessor(), output.getmRelays());
        dumpBasicConfig(DataAccessorFactory.getInstance().getSolenoidAccessor(), output.getmSolenoids());

        dumpEncoderConfig(DataAccessorFactory.getInstance().getEncoderAccessor(), output.getmEncoders());
        dumpPwmConfig(DataAccessorFactory.getInstance().getSpeedControllerAccessor(), output.getmPwm());
        dumpSimulatorComponents(DataAccessorFactory.getInstance().getSimulatorDataAccessor(), output.getmSimulatorComponents());

        output.setmDefaultI2CWrappers(DataAccessorFactory.getInstance().getSimulatorDataAccessor().getDefaultI2CWrappers());
        output.setmDefaultSpiWrappers(DataAccessorFactory.getInstance().getSimulatorDataAccessor().getDefaultSpiWrappers());

        return output;
    }

    protected void dumpBasicConfig(IBasicSensorActuatorWrapperAccessor aAccessor, List<BasicModuleConfig> aOutputList)
    {
        for (int portHandle : aAccessor.getPortList())
        {
            BasicModuleConfig config = new BasicModuleConfig(portHandle, aAccessor.getName(portHandle));
            aOutputList.add(config);
        }
    }

    protected void dumpEncoderConfig(EncoderWrapperAccessor aAccessor, List<EncoderConfig> aOutputList)
    {
        for (int portHandle : aAccessor.getPortList())
        {
            EncoderConfig config = new EncoderConfig(portHandle, aAccessor.getName(portHandle), aAccessor.getHookedUpId(portHandle));
            aOutputList.add(config);
        }
    }

    protected void dumpPwmConfig(SpeedControllerWrapperAccessor aAccessor, List<PwmConfig> aOutputList)
    {
        for (int portHandle : aAccessor.getPortList())
        {
            PwmConfig config = new PwmConfig(portHandle, aAccessor.getName(portHandle), getMotorSimConfig(aAccessor, portHandle),
                    getMotorModel(aAccessor, portHandle));
            aOutputList.add(config);
        }
    }

    protected DcMotorModelConfig.FactoryParams getMotorModel(SpeedControllerWrapperAccessor aAccessor, int aPortHandle)
    {
        DcMotorModelConfig motorConfig = aAccessor.getMotorConfig(aPortHandle);
        return motorConfig == null ? null : motorConfig.mFactoryParams;
    }

    protected void dumpSimulatorComponents(SimulatorDataAccessor aAccessor, List<Object> aOutputList)
    {
        for (Object config : aAccessor.getSimulatorComponentConfigs())
        {
            aOutputList.add(config);
        }
    }

    protected IMotorSimulatorConfig getMotorSimConfig(SpeedControllerWrapperAccessor aAccessor, int aPortHandle)
    {
        MotorSimType simType = aAccessor.getMotorSimType(aPortHandle);
        switch (simType)
        {
        case Simple:
            return aAccessor.getMotorSimSimpleModelConfig(aPortHandle);
        case StaticLoad:
            return aAccessor.getMotorSimStaticModelConfig(aPortHandle);
        case GravitationalLoad:
            return aAccessor.getMotorSimGravitationalModelConfig(aPortHandle);
        case RotationalLoad:
            return aAccessor.getMotorSimRotationalModelConfig(aPortHandle);

        case None:
        default:
            break;
        }
        return null;
    }
}
