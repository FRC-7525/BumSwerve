package frc.robot.pioneersLib.CI;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public class Crash {
    public static void main(String[] args) {
        try {
            // Setup
            HAL.initialize(500, 0);
            
            runSimTests();
            
            System.out.println("All tests passed");
            System.exit(0);
        } catch (Exception e) {
            System.err.println("Test failed: " + e.getMessage());
            e.printStackTrace();
            System.exit(1);
        }
    }

    private static void runSimTests() throws InterruptedException {
        boolean crashesAuto = false;
        boolean crashesTele = false;

        // Test autonomous
        System.out.println("Testing auto");
        setMode(true);
        Thread.sleep(500);
        setMode(false);

        if (checkForErrors()) {
            crashesAuto = true;
        }

        // Test teleop
        System.out.println("Testing teleop");
        setMode(false);
        Thread.sleep(500);
        setMode(false);

        if (checkForErrors()) {
            crashesTele = true;
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

    private static void setMode(boolean autonomous) {
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(autonomous);
    }

    private static boolean checkForErrors() {
        return DriverStationSim.getEStop() || !DriverStationSim.getDsAttached();
    }
}