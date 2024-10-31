// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.pioneersLib.CI.Crash;
import frc.robot.subsystems.manager.Manager;

public class Robot extends LoggedRobot {

	private Manager manager;

	@Override
	public void robotInit() {
		Logger.addDataReceiver(new NT4Publisher());
		Logger.start();
		DriverStation.silenceJoystickConnectionWarning(true);

		manager = new Manager();

		if ("Crash".equals(System.getenv("CI_NAME"))) {
			Crash.getInstance(this).run();
			System.out.println("lalala");
		}
	}

	@Override
	public void robotPeriodic() {
		manager.periodic();

		if ("Crash".equals(System.getenv("CI_NAME"))) {
			Crash.getInstance(this).periodic();
			System.out.println("lalala");
		}
	}

	@Override
	public void autonomousInit() {
		// System.out.println("AUTO ENABLED");
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		// System.out.println("TELE ENABLED");
		TalonFX lala = new TalonFX(-5);
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
		manager.periodic();
	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void simulationInit() {
	}

	@Override
	public void simulationPeriodic() {
	}
}
