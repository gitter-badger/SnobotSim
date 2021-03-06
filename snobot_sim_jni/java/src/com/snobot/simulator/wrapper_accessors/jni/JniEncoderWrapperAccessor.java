
package com.snobot.simulator.wrapper_accessors.jni;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import com.snobot.simulator.jni.module_wrapper.EncoderWrapperJni;
import com.snobot.simulator.wrapper_accessors.EncoderWrapperAccessor;

public class JniEncoderWrapperAccessor implements EncoderWrapperAccessor
{
    
    @Override
    public void setName(int aPort, String aName)
    {
        EncoderWrapperJni.setName(aPort, aName);
    }
    
    @Override
    public String getName(int aPort)
    {
        return EncoderWrapperJni.getName(aPort);
    }

    @Override
    public boolean getWantsHidden(int aPort)
    {
        return EncoderWrapperJni.getWantsHidden(aPort);
    }
    
    @Override
    public boolean connectSpeedController(int aEncoderHandle, int aSpeedControllerHandle)
    {
        return EncoderWrapperJni.connectSpeedController(aEncoderHandle, aSpeedControllerHandle);
    }

    @Override
    public boolean isHookedUp(int aPort)
    {
        return EncoderWrapperJni.isHookedUp(aPort);
    }
    
    @Override
    public int getHookedUpId(int aPort)
    {
        return EncoderWrapperJni.getHookedUpId(aPort);
    }

    @Override
    public double getRaw(int aPort)
    {
        return EncoderWrapperJni.getRaw(aPort);
    }
    
    @Override
    public double getDistance(int aPort)
    {
        return EncoderWrapperJni.getDistance(aPort);
    }
    
    @Override
    public List<Integer> getPortList()
    {
        return IntStream.of(EncoderWrapperJni.getPortList()).boxed().collect(Collectors.toList());
    }
}
