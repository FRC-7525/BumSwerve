// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.manager.Manager;

public class Robot extends LoggedRobot {

	private Manager manager;
	private SendableChooser<String> autoChooser;
	private Command autonomousCommand;

	@Override
	public void robotInit() {
		Logger.addDataReceiver(new NT4Publisher());
		Logger.start();
		DriverStation.silenceJoystickConnectionWarning(true);

		manager = new Manager();

		autoChooser = new SendableChooser<String>();
		autoChooser.setDefaultOption("0: Test Auto", "Test Auto");
		
		autoChooser.addOption("0: Test Auto", "Test Auto");

		SmartDashboard.putData("Auto autoChooser", autoChooser);
	}

	@Override
	public void robotPeriodic() {
		manager.periodic();

		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = new PathPlannerAuto(autoChooser.getSelected());

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
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
