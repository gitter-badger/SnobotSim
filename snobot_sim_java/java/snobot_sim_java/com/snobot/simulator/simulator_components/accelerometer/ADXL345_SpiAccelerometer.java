package com.snobot.simulator.simulator_components.accelerometer;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.snobot.simulator.jni.SensorFeedbackJni;
import com.snobot.simulator.simulator_components.ISpiWrapper;

public class ADXL345_SpiAccelerometer implements ISpiWrapper
{
    private static double sLSB = 0.00390625;

    protected final ThreeAxisAccelerometer mDataContainer;
    protected final int mNativePort;

    public ADXL345_SpiAccelerometer(int aPort)
    {
        mDataContainer = new ThreeAxisAccelerometer(aPort * 75, "SPI Accel ");
        mNativePort = aPort;
    }

    @Override
    public void handleRead()
    {
        ByteBuffer lastWriteValue = ByteBuffer.allocateDirect(4);
        SensorFeedbackJni.getSpiLastWrite(mNativePort, lastWriteValue, 4);
        lastWriteValue.rewind();
        int lastWrittenAddress = lastWriteValue.get() & 0xF;
        ByteBuffer buffer = ByteBuffer.allocateDirect(10);
        buffer.put((byte) 0);
        buffer.order(ByteOrder.LITTLE_ENDIAN);

        boolean includeAll = lastWrittenAddress == 0x02;

        if (lastWrittenAddress >= 0x2 && lastWrittenAddress <= 0x6)
        {
            if (lastWrittenAddress == 0x02)
            {
                short value = (short) (mDataContainer.getX() / sLSB);
                buffer.putShort(value);
            }

            if (lastWrittenAddress == 0x04 || includeAll)
            {
                short value = (short) (mDataContainer.getY() / sLSB);
                buffer.putShort(value);
            }

            if (lastWrittenAddress == 0x06 || includeAll)
            {
                short value = (short) (mDataContainer.getZ() / sLSB);
                buffer.putShort(value);
            }

            SensorFeedbackJni.setSpiValueForRead(mNativePort, buffer, buffer.capacity());
        }
        else
        {
            System.err.println("Unsupported write address " + lastWrittenAddress);
        }
    }

}
