// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.pioneersLib.CI.Crash;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
    private Main() {}

    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>If you change your main robot class, change the parameter type.
     */
    public static void main(String... args) {
        if (isRunningInCI()) {
            // If running in CI, run the Crash class
            Crash.main(args);
        } else {
            // Otherwise, start the normal robot
            RobotBase.startRobot(Robot::new);
        }
    }

    /**
     * Checks if the code is running in a CI environment. (uh or like its just for the crash check but wtv)
     *
     * @return true if running in a CI environment; false otherwise.
     */
    private static boolean isRunningInCI() {
        // Check if CI_NAME environment variable is set to "Crash"
        return "Crash".equals(System.getenv("CI_NAME"));
    }
}
