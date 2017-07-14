package com.snobot.simulator.robot_container;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import com.snobot.simulator.JniLibraryResourceLoader;

public class CppRobotContainer implements IRobotClassContainer
{
    private String mRobotClassName;
    private Class<?> mJniClass;

    public CppRobotContainer(String aRobotClassName)
    {
        mRobotClassName = aRobotClassName;
    }

    @Override
    public void constructRobot()
            throws InstantiationException, IllegalAccessException, ClassNotFoundException, NoSuchMethodException, SecurityException,
            IllegalArgumentException, InvocationTargetException
    {
        mJniClass = Class.forName(mRobotClassName);

        String libraryName = (String) mJniClass.getMethod("getLibraryName").invoke(null);

        JniLibraryResourceLoader.loadLibrary("ntcore");
        JniLibraryResourceLoader.loadLibrary("wpiutil");
        JniLibraryResourceLoader.loadLibrary("wpilibc");
        JniLibraryResourceLoader.loadLibrary("CTRLibDriver");
        JniLibraryResourceLoader.loadLibrary("CtrCpp");
        JniLibraryResourceLoader.loadLibrary(libraryName);

        Method method = mJniClass.getMethod("createRobot");
        method.invoke(null);
    }

    @Override
    public void startCompetition()
            throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
    {
        Method method = mJniClass.getMethod("startCompetition");
        method.invoke(null);
    }
}
