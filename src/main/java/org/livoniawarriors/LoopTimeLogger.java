package org.livoniawarriors;

import java.lang.reflect.Field;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;

public class LoopTimeLogger implements Runnable {
    private NetworkTable table;
    private Map<String, Long> robotEpochs;
    private Map<String, Long> cmdEpochs;

    public LoopTimeLogger(Robot robot, NetworkTable table) {
        /**
         * By registering as a task, we get:
         * Init Calls
         * Periodic calls (both auto/teleop and Robot)
         * 
         * You don't get the time in updating SmartDashboard, LiveWindow, Shuffleboard,
         * and Simulation
         */
        UtilFunctions.addPeriodic(this, Robot.kDefaultPeriod, 0);
        this.table = table;
        try {
            // read the robot class for the watchdog
            Class<?> f = robot.getClass().getSuperclass().getSuperclass();
            Field field = f.getDeclaredField("m_watchdog");
            field.setAccessible(true); // Make it accessible so you can access it
            Watchdog watchDog = (Watchdog) field.get(robot); // At last it's yours.
            robotEpochs = watchToMap(watchDog);

            CommandScheduler cs = CommandScheduler.getInstance();
            f = cs.getClass();
            field = f.getDeclaredField("m_watchdog");
            field.setAccessible(true);
            watchDog = (Watchdog) field.get(cs);
            cmdEpochs = watchToMap(watchDog);
        } catch (Exception e) {
            DriverStation.reportError("Task Timing could NOT be initialized.", false);
        }
    }

    @SuppressWarnings("unchecked")
    public Map<String, Long> watchToMap(Watchdog watchDog)
            throws IllegalArgumentException, IllegalAccessException, NoSuchFieldException, SecurityException {
        // from the watchdog, get the tracer
        var watchClass = watchDog.getClass();
        Field field = watchClass.getDeclaredField("m_tracer");
        field.setAccessible(true);
        Tracer tracer = (Tracer) field.get(watchDog);

        // from the tracer. get the records
        var tracerClass = tracer.getClass();
        field = tracerClass.getDeclaredField("m_epochs");
        field.setAccessible(true);
        return (Map<String, Long>) field.get(tracer);
    }

    @Override
    public void run() {
        if (robotEpochs != null) {
            double totalTime = 0.0;
            for (String key : robotEpochs.keySet()) {
                NetworkTableEntry entry = table.getEntry(key);
                double time = robotEpochs.get(key) / 1000f;
                entry.setDouble(time);
                totalTime += time;
            }
            table.getEntry("Loop Time").setDouble(totalTime);
        }

        if (cmdEpochs != null) {
            double cmdTime = 0.0;
            for (String key : cmdEpochs.keySet()) {
                NetworkTableEntry entry = table.getEntry(key);
                double time = cmdEpochs.get(key) / 1000f;
                entry.setDouble(time);
                cmdTime += time;
            }
            table.getEntry("Scheduler Time").setDouble(cmdTime);
        }
    }
}
