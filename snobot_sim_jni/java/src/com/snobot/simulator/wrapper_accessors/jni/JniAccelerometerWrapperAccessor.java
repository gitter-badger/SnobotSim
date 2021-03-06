
package com.snobot.simulator.wrapper_accessors.jni;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import com.snobot.simulator.jni.module_wrapper.AccelerometerWrapperJni;
import com.snobot.simulator.wrapper_accessors.AccelerometerWrapperAccessor;

public class JniAccelerometerWrapperAccessor implements AccelerometerWrapperAccessor
{
    @Override
    public void register(int aPort, String aName)
    {
        AccelerometerWrapperJni.register(aPort, aName);
    }
    
    @Override
    public void setName(int aPort, String aName)
    {
        AccelerometerWrapperJni.setName(aPort, aName);
    }
    
    @Override
    public String getName(int aPort)
    {
        return AccelerometerWrapperJni.getName(aPort);
    }

    @Override
    public boolean getWantsHidden(int aPort)
    {
        return AccelerometerWrapperJni.getWantsHidden(aPort);
    }
    
    @Override
    public double getAcceleration(int aPort)
    {
        return AccelerometerWrapperJni.getAcceleration(aPort);
    }

    @Override
    public void setAcceleration(int aPort, double aAcceleration)
    {
        AccelerometerWrapperJni.setAcceleration(aPort, aAcceleration);
    }
    
    @Override
    public List<Integer> getPortList()
    {
        return IntStream.of(AccelerometerWrapperJni.getPortList()).boxed().collect(Collectors.toList());
    }
}
