package com.snobot.test.utilities;

import java.io.File;

import org.junit.Before;

import com.snobot.simulator.jni.SnobotSimulatorJni;
import com.snobot.simulator.wrapper_accessors.DataAccessorFactory;
import com.snobot.simulator.wrapper_accessors.jni.JniDataAccessor;

import edu.wpi.first.wpilibj.RobotBase;

public class BaseSimulatorTest
{
    private static boolean INITIALIZED = false;
    protected static final double DOUBLE_EPSILON = .0002;

    private void delete(File path)
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
            DataAccessorFactory.setAccessor(new JniDataAccessor());
            SnobotSimulatorJni.initializeLogging(1);

            File directory = new File("test_output");
            if (directory.exists())
            {
                delete(directory);
            }
            directory.mkdirs();
        }

        SnobotSimulatorJni.reset();
        RobotBase.initializeHardwareConfiguration();
    }

    protected void simulateForTime(double aSeconds, Runnable task)
    {
        simulateForTime(aSeconds, .02, task);
    }

    protected void simulateForTime(double aSeconds, double aUpdatePeriod, Runnable aTask)
    {
        double update_frequency = 1 / aUpdatePeriod;

        for (int i = 0; i < update_frequency * aSeconds; ++i)
        {
            aTask.run();
            DataAccessorFactory.getInstance().getSimulatorDataAccessor().updateSimulatorComponents(aUpdatePeriod);
        }
    }
}
