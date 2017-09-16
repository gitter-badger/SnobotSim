package com.snobot.simulator;

import java.io.File;
import java.io.FileInputStream;
import java.lang.reflect.InvocationTargetException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Properties;

import javax.swing.JFrame;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import com.snobot.simulator.gui.SimulatorFrame;
import com.snobot.simulator.joysticks.IMockJoystick;
import com.snobot.simulator.joysticks.JoystickFactory;
import com.snobot.simulator.robot_container.CppRobotContainer;
import com.snobot.simulator.robot_container.IRobotClassContainer;
import com.snobot.simulator.robot_container.JavaRobotContainer;
import com.snobot.simulator.robot_container.PythonRobotContainer;
import com.snobot.simulator.wrapper_accessors.DataAccessorFactory;
import com.snobot.simulator.wrapper_accessors.SimulatorDataAccessor.SnobotLogLevel;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Simulator
{
    private final String USER_CONFIG_DIRECTORY;
    private final String PROPERTIES_FILE;

    private String mSimulatorClassName; // The name of the class that represents the simulator
    private String mSimulatorConfig;

    private IRobotClassContainer mRobot; // The robot code to run
    private ASimulator mSimulator; // The robot code to run

    public Simulator(SnobotLogLevel aLogLevel, File aPluginDirectory, String aUserConfigDir) throws Exception
    {
        DataAccessorFactory.getInstance().getSimulatorDataAccessor().setLogLevel(aLogLevel);

        PluginSniffer sniffer = new PluginSniffer();
        sniffer.loadPlugins(aPluginDirectory);

        USER_CONFIG_DIRECTORY = aUserConfigDir;
        PROPERTIES_FILE = USER_CONFIG_DIRECTORY + "simulator_config.properties";

        File config_dir = new File(aUserConfigDir);
        if (!Files.exists(config_dir.toPath()))
        {
            config_dir.mkdir();
        }
    }

    private void loadConfig(String aFile)
    {

        try
        {
            if (!Files.exists(Paths.get(aFile)))
            {
                System.err.println("Could not read properties file, will use defaults and will overwrite the file if it exists");

                if (!JniLibraryResourceLoader.copyResourceFromJar("/com/snobot/simulator/config/default_properties.properties",
                        new File(PROPERTIES_FILE), false))
            	{
            		throw new RuntimeException("Could not copy properties file!  Have to exit!");
            	}
            }

            Properties p = new Properties();
            p.load(new FileInputStream(new File(aFile)));

            String robotClassName = p.getProperty("robot_class");
            mSimulatorClassName = p.getProperty("simulator_class");
            mSimulatorConfig = p.getProperty("simulator_config");

            String robotType = p.getProperty("robot_type");
            if (robotType == null || robotType.equals("java"))
            {
                mRobot = new JavaRobotContainer(robotClassName);
            }
            else if (robotType.equals("cpp"))
            {
                mRobot = new CppRobotContainer(robotClassName);
            }
            else if (robotType.equals("python"))
            {
                mRobot = new PythonRobotContainer(robotClassName);
            }
            else
            {
                throw new RuntimeException("Unsuppored robot type " + robotType);
            }

            NetworkTable.setPersistentFilename(USER_CONFIG_DIRECTORY + robotClassName + ".preferences.ini");
        }
        catch (Exception e)
        {
            e.printStackTrace();
            System.err.println("Could not read properties file");
        }
    }

    private void createRobot() throws InstantiationException, IllegalAccessException, ClassNotFoundException, NoSuchMethodException,
            SecurityException, IllegalArgumentException, InvocationTargetException
    {
        Logger.getLogger(Simulator.class).log(Level.INFO, "Starting Robot Code");

        mRobot.constructRobot();
    }

    public void startSimulation()
            throws InstantiationException, IllegalAccessException, ClassNotFoundException, NoSuchMethodException, SecurityException,
            IllegalArgumentException, InvocationTargetException
    {
        loadConfig(PROPERTIES_FILE);

        sendJoystickUpdate();
        createSimulator();
        createRobot();

        Thread robotThread = new Thread(createRobotThread(), "RobotThread");
        Thread simulation_thread = new Thread(createSimulationThread(), "SimulatorThread");

        simulation_thread.start();
        robotThread.start();
    }

    protected void setFrameVisible(SimulatorFrame frame)
    {
        frame.pack();
        frame.setVisible(true);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }

    private void sendJoystickUpdate()
    {
        IMockJoystick[] joysticks = JoystickFactory.get().getAll();
        for (int i = 0; i < joysticks.length; ++i)
        {
            IMockJoystick joystick = joysticks[i];
            DataAccessorFactory.getInstance().getSimulatorDataAccessor().setJoystickInformation(i, joystick.getAxisValues(), joystick.getPovValues(),
                    joystick.getButtonCount(),
                    joystick.getButtonMask());
        }
    }

    private void createSimulator()
    {
        try
        {
            if (mSimulatorClassName != null && !mSimulatorClassName.isEmpty())
            {
                mSimulator = (ASimulator) Class.forName(mSimulatorClassName).newInstance();
            }
            else
            {
                mSimulator = new ASimulator();
            }

        }
        catch (ClassNotFoundException | InstantiationException | IllegalAccessError | IllegalAccessException e)
        {
            throw new RuntimeException("Could not find simulator class " + mSimulatorClassName);
        }
    }

    private Runnable createSimulationThread()
    {
        return new Runnable()
        {

            @Override
            public void run()
            {
                try
                {
                    DataAccessorFactory.getInstance().getSimulatorDataAccessor().waitForProgramToStart();

                    if (mSimulator != null)
                    {
                        mSimulator.createSimulatorComponents(mSimulatorConfig);
                        mSimulator.setRobot(mRobot);
                        Logger.getLogger(Simulator.class).log(Level.INFO, "Created simulator : " + mSimulatorClassName);
                    }

                    SimulatorFrame frame = new SimulatorFrame(mSimulatorConfig);
                    setFrameVisible(frame);

                    while (true)
                    {
                        DataAccessorFactory.getInstance().getSimulatorDataAccessor().waitForNextUpdateLoop();

                        mSimulator.update();
                        frame.updateLoop();
                        sendJoystickUpdate();
                    }
                }
                catch (Throwable e)
                {
                    System.err.println("Encountered fatal error, will exit.  Error: " + e.getMessage());
                    e.printStackTrace();
                    System.exit(1);
                }
            }
        };
    }

    private Runnable createRobotThread()
    {
        return new Runnable()
        {

            @Override
            public void run()
            {

                try
                {
                    mRobot.startCompetition();
                }
                catch (UnsatisfiedLinkError e)
                {
                    e.printStackTrace();
                    System.err.println("\n\n\n\n");
                    System.err.println("Unsatisfied link error.  This likely means that there is a native "
                            + "call in WpiLib or the NetworkTables libraries.  Please tell PJ so he can mock it out.\n\nError Message: " + e);

                    System.exit(-1);
                }
                catch (Exception e)
                {
                    Logger.getLogger(Simulator.class).log(Level.FATAL, "Unexpected exception, shutting down simulator");
                    e.printStackTrace();
                    System.exit(-1);
                }
            }
        };
    }
}
