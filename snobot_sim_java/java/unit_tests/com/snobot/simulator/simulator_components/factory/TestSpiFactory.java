package com.snobot.simulator.simulator_components.factory;

import java.util.Collection;
import java.util.Map;

import org.junit.Assert;
import org.junit.Test;

import com.snobot.simulator.wrapper_accessors.DataAccessorFactory;
import com.snobot.test.utilities.BaseSimulatorTest;

import edu.wpi.first.wpilibj.ADXL345_SPI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;

public class TestSpiFactory extends BaseSimulatorTest
{

    @Test
    public void testAvailableDataTypes()
    {
        DataAccessorFactory.getInstance().getSimulatorDataAccessor().reset();
        Collection<String> available = DataAccessorFactory.getInstance().getSimulatorDataAccessor().getAvailableSpiSimulators();

        Assert.assertEquals(4, available.size());
        Assert.assertTrue(available.contains("NavX"));
        Assert.assertTrue(available.contains("ADXRS450"));
        Assert.assertTrue(available.contains("ADXL345"));
        Assert.assertTrue(available.contains("ADXL362"));
    }

    @Test
    public void testInvalidType()
    {
        DataAccessorFactory.getInstance().getSimulatorDataAccessor().reset();
        DataAccessorFactory.getInstance().getSimulatorDataAccessor().setDefaultI2CSimulator(0, "DoesntExist");

        new ADXL345_SPI(SPI.Port.kOnboardCS0, Range.k2G);
    }

    @Test
    public void testDefaultSpiWrappers()
    {
        DataAccessorFactory.getInstance().getSimulatorDataAccessor().setDefaultSpiSimulator(0, "TestA");
        DataAccessorFactory.getInstance().getSimulatorDataAccessor().setDefaultSpiSimulator(1, "TestB");
        Map<Integer, String> defaults = DataAccessorFactory.getInstance().getSimulatorDataAccessor().getDefaultSpiWrappers();

        Assert.assertEquals(2, defaults.size());
        Assert.assertEquals("TestA", defaults.get(0));
        Assert.assertEquals("TestB", defaults.get(1));
    }
}
