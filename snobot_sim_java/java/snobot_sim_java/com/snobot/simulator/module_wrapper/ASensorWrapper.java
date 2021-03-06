package com.snobot.simulator.module_wrapper;

public class ASensorWrapper implements ISensorWrapper
{
    protected String mName;
    protected boolean mWantsHidden;

    public ASensorWrapper(String aName)
    {
        mName = aName;
    }

    /* (non-Javadoc)
     * @see com.snobot.simulator.module_wrapper.ISensorWrapper#getName()
     */
    @Override
    public String getName()
    {
        return mName;
    }

    /* (non-Javadoc)
     * @see com.snobot.simulator.module_wrapper.ISensorWrapper#setName(java.lang.String)
     */
    @Override
    public void setName(String aName)
    {
        mName = aName;
    }

    /* (non-Javadoc)
     * @see com.snobot.simulator.module_wrapper.ISensorWrapper#getWantsHidden()
     */
    @Override
    public boolean getWantsHidden()
    {
        return mWantsHidden;
    }

    /* (non-Javadoc)
     * @see com.snobot.simulator.module_wrapper.ISensorWrapper#setWantsHidden(boolean)
     */
    @Override
    public void setWantsHidden(boolean aVisible)
    {
        mWantsHidden = aVisible;
    }
}
