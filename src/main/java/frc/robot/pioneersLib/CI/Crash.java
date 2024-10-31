package frc.robot.pioneersLib.CI;

import org.littletonrobotics.junction.inputs.LoggedDriverStation.DriverStationInputs;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.hal.simulation.NotifyCallback;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Robot;

public class Crash {

    private Robot robot;
    private static Crash instance;

    public Crash(Robot robot) {
        this.robot = robot;

        DriverStationSim.registerAutonomousCallback(new NotifyCallback() {
            @Override
            public void callback(String name, HALValue value) {
                robot.autonomousInit();
            }
        }, false);

        DriverStationSim.registerEnabledCallback(new NotifyCallback() {
            @Override
            public void callback(String name, HALValue value) {
                robot.teleopInit();
            }
        }, false);
    }

    public static Crash getInstance(Robot robot) {
        if (instance == null) {
            instance = new Crash(robot);
        }
        return instance;
    }

    public void run() {
        try {
            runSimTests();
            
            System.out.println("All tests passed");
            System.exit(0);
        } catch (Exception e) {
            System.err.println("Test failed: " + e.getMessage());
            e.printStackTrace();
            System.exit(1);
        }
    }

    public void periodic() {
        System.out.println(DriverStationSim.getEnabled());
        if (DriverStationSim.getEnabled() && !DriverStationSim.getAutonomous()) {
            robot.teleopPeriodic();
        } else if (DriverStationSim.getAutonomous() && DriverStationSim.getEnabled()) {
            robot.autonomousPeriodic();
        } else {
            robot.disabledPeriodic();
        }
    }

    private void runSimTests() throws InterruptedException {
        boolean crashesAuto = false;
        boolean crashesTele = false;

        // Test autonomous
        System.out.println("Testing auto");
        setMode(true);
        Thread.sleep(3000);
        setMode(false);

        if (checkForErrors()) {
            crashesAuto = true;
            System.out.println("auto crash");
        }

        // Test teleop
        System.out.println("Testing teleop");
        setMode(false);
        Thread.sleep(3000);
        setMode(false);

        if (checkForErrors()) {
            crashesTele = true;
            System.out.println("teleop crash");
        }

        // throw errors
        if (crashesAuto && crashesTele) {
            throw new RuntimeException("Code crashes in both auto and teleop");
        } else if (crashesAuto) {
            throw new RuntimeException("Code crashes in autonomous");
        } else if (crashesTele) {
            throw new RuntimeException("Code crashes in teleop");
        }
    }

    public void setMode(boolean autonomous) {
        DriverStationSim.setEnabled(true);

        DriverStationSim.setAutonomous(autonomous);
    }

    // Duh, if ur not enabled it's not bouta work
    private boolean checkForErrors() {
        return !DriverStationSim.getEnabled();
    }
}