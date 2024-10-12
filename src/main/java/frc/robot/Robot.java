// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;

public class Robot extends LoggedRobot {
	Drive drive;
	CommandScheduler commandScheduler;

	@Override
	public void robotInit() {
		this.drive = new Drive();
		Logger.addDataReceiver(new NT4Publisher());
		Logger.addDataReceiver(new WPILOGWriter("Logs"));
		Logger.start();
		commandScheduler = CommandScheduler.getInstance();
		
		Logger.recordOutput("Drive/SysIdState", "none");
		Logger.addDataReceiver(null);
	}
	
	@Override
	public void robotPeriodic() {
		drive.periodic();
		drive.runState();
		commandScheduler.run();
	}

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
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
