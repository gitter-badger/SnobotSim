package com.snobot.test.utilities;

import java.io.File;

import org.junit.After;
import org.junit.Before;

import com.snobot.simulator.DefaultDataAccessorFactory;
import com.snobot.simulator.wrapper_accessors.DataAccessorFactory;
import com.snobot.simulator.wrapper_accessors.SimulatorDataAccessor.SnobotLogLevel;

import edu.wpi.first.wpilibj.RobotBase;

public class BaseSimulatorTest
{
    private static boolean INITIALIZED = false;
    protected static final double DOUBLE_EPSILON = .00001;

    protected final void delete(File path)
    {
        File[] l = path.listFiles();
        for (File f : l)
        {
            if (f.isDirectory())
            {
                delete(f);
            }
            else
            {
                f.delete();
            }
        }
        path.delete();
    }

    @Before
    public void setup()
    {
        if (!INITIALIZED)
        {
            DefaultDataAccessorFactory.initalize();
            DataAccessorFactory.getInstance().getSimulatorDataAccessor().setLogLevel(SnobotLogLevel.DEBUG);

            File directory = new File("test_output");
            if (directory.exists())
            {
                delete(directory);
            }
            directory.mkdirs();
            INITIALIZED = true;
        }

        DataAccessorFactory.getInstance().getSimulatorDataAccessor().reset();
        RobotBase.initializeHardwareConfiguration();
    }

    @After
    public void cleanup()
    {

    }
}
