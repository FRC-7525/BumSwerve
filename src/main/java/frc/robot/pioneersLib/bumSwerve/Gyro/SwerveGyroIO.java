// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.pioneersLib.bumSwerve.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveGyroIO {
	@AutoLog
	public static class SwerveGyroIOInputs {

		public boolean connected = false;
		public Rotation2d yawPosition = new Rotation2d();
		public double[] odometryYawTimestamps = new double[] {};
		public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
		public double yawVelocityRPS = 0.0;
	}

	/**
	 * Zero the gyro's rotation position
	 */
	public default void zero() {}

	/**
	 * Used to set gyro rotation in sim
	 * @param rotation Rotation3d object with x, y, z values
	 */
	public default void setAngle(Rotation3d rotation) {}

	/**
	 * Updates the inputs class with the current gyro data
	 * @param inputs Inputs class that's automatically logged by Akit from the Autolog annotation
	 */
	public default void updateInputs(SwerveGyroIOInputs inputs) {}
}
