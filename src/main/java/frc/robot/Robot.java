// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.pioneersLib.simulation.SimulatedArena;
import frc.robot.pioneersLib.simulation.seasonSpecific.CrescendoNoteOnField;
import frc.robot.subsystems.manager.Manager;

public class Robot extends LoggedRobot {

	private Manager manager;

	@Override
	public void robotInit() {
		Logger.addDataReceiver(new NT4Publisher());
		Logger.start();
    SimulatedArena.getInstance();
    // SimulatedArena.overrideInstance(SimulatedArena newInstance); Maybe needed?
	SimulatedArena.getInstance().addGamePiece(new CrescendoNoteOnField(new Translation2d(3, 3)));


		manager = new Manager();
	}

	@Override
	public void robotPeriodic() {
		manager.periodic();
		SimulatedArena.getInstance().simulationPeriodic();
		SimulatedArena.getInstance().addGamePiece(new CrescendoNoteOnField(new Translation2d(3, 3)));
		List<Pose3d> notesPoses = SimulatedArena.getInstance().getGamePiecesByType("Note");
		String output = ""; 
		for (Pose3d notePose : notesPoses) { 
			output += notePose.toString() + "\n";
		} 
		Logger.recordOutput("FieldSimulation/NotesPositions", output.getBytes());

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
    SimulatedArena.getInstance().simulationPeriodic();
	}

}
