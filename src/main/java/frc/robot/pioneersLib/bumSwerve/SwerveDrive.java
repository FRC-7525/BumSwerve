package frc.robot.pioneersLib.bumSwerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIO;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIOInputsAutoLogged;

public class SwerveDrive {
	static final Lock odometryLock = new ReentrantLock();
	private final SwerveGyroIOInputsAutoLogged gyroInputs = new SwerveGyroIOInputsAutoLogged();

	// Constants
	private static final double OPTIMAL_VOLTAGE = 12.0;
	private static final double GRAVITY = 9.81;
	private static final double HEADING_CORRECTION_DEADBAND = 0.05;
	private static final double DT_TIME_SECONDS = 0.02;

	private PIDController headingCorrectionController;
	private SwerveGyroIO gyroIO;
	private final SwerveModule[] modules; // FL, FR, BL, BR
	private Rotation2d rawGyroRotation = new Rotation2d();

	private static double trackWidthX;
	private static double trackWidthY;

	public static double maxSpeed;
	public static double wheelRadius;
	private int numModules;
	private boolean isSim;
	private Rotation2d lastHeading;

	private SwerveDriveKinematics kinematics;
	private SwerveDrivePoseEstimator poseEstimator;
	private SwerveModulePosition[] lastModulePositions = // "for delta tracking", no idea what this does :( helps pose
															// estimator ig
			new SwerveModulePosition[] {
					new SwerveModulePosition(),
					new SwerveModulePosition(),
					new SwerveModulePosition(),
					new SwerveModulePosition(),
			};

	/**
	 * Creates a new SwerveDrive object, if configs are messed up
	 * your robot wont drive correctly. Take time to make measurments.
	 * 
	 * @param trackWidthX Distance between the left and right wheels in meters
	 * @param trackWidthY Distance between the front and back wheels in meters
	 * @param modules     Array of SwerveModule objects in the order FL, FR, BL, BR
	 * @param gyroIO      Gyro object
	 * @param maxSpeed    Max speed of the robot in m/s
	 * @param wheelRadius Radius of the wheels in meters
	 * @param isSim       If or not the drivetrain is in simulation
	 */
	public SwerveDrive(double trackWidthX, double trackWidthY, SwerveModule[] modules, SwerveGyroIO gyroIO,
			double maxSpeed, double wheelRadius, boolean isSim) {
		SwerveDrive.trackWidthX = trackWidthX;
		SwerveDrive.trackWidthY = trackWidthY;
		OdometryThread.getInstance().start();

		kinematics = new SwerveDriveKinematics(getModuleTranslations());
		poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
		headingCorrectionController = new PIDController(0, 0, 0);
		lastHeading = new Rotation2d();

		this.gyroIO = gyroIO;
		SwerveDrive.maxSpeed = maxSpeed;
		SwerveDrive.wheelRadius = wheelRadius;

		this.modules = modules;
		this.isSim = isSim;
		this.numModules = modules.length;

		SimpleMotorFeedforward driveFF = createDriveFeedforward(OPTIMAL_VOLTAGE, maxSpeed, 1.19);

		for (var module : modules) {
			module.configureDriveFF(driveFF.ks, driveFF.kv, driveFF.ka);
		}
	}

	@AutoLogOutput(key = "Robot Pose")
	public Pose2d getRobotPose() {
		return poseEstimator.getEstimatedPosition();
	}

	/**
	 * Creates a new simple FF object
	 * 
	 * @param optimalVoltage                 Optimal voltage (typically 12V)
	 * @param maxSpeed                       Max module speed
	 * @param wheelGripCoefficientOfFriction Coefficient of friction of the wheels
	 */
	public static SimpleMotorFeedforward createDriveFeedforward(
			double optimalVoltage,
			double maxSpeed,
			double wheelGripCoefficientOfFriction) {
		double kv = optimalVoltage / maxSpeed;
		/// ^ Volt-seconds per meter (max voltage divided by max speed)
		double ka = optimalVoltage / calculateMaxAcceleration(wheelGripCoefficientOfFriction);
		/// ^ Volt-seconds^2 per meter (max voltage divided by max accel)
		return new SimpleMotorFeedforward(0, kv, ka);
	}

	/**
	 * @param cof Coefficient of friction of the wheels
	 * @return Max possible acceleration
	 */
	public static double calculateMaxAcceleration(double cof) {
		return cof * GRAVITY;
	}

	/**
	 * Drives the robot.
	 * 
	 * @param xSupplier            X speed supplier, in range 0-1
	 * @param ySupplier            Y speed supplier, in range 0-1
	 * @param omegaSupplier        Omega supplier, in range 0-1
	 * @param fieldRelative        Whether the speeds are field relative
	 * @param useHeadingCorrection Whether to use heading correction
	 */
	public void drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier,
		boolean fieldRelative, boolean useHeadingCorrection) {
		boolean isFlipped = DriverStation.getAlliance().isPresent() &&
				DriverStation.getAlliance().get() == Alliance.Red;

		ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
				xSupplier.getAsDouble() * getMaxSpeed(),
				ySupplier.getAsDouble() * getMaxSpeed(),
				omegaSupplier.getAsDouble() * getMaxAngularVelocity(),
				getRobotRotation());
		ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				xSupplier.getAsDouble() * getMaxSpeed(), 
				ySupplier.getAsDouble() * getMaxSpeed(),
				omegaSupplier.getAsDouble() * (getMaxAngularVelocity()),
				isFlipped ? getRobotRotation().plus(new Rotation2d(Math.PI)) : getRobotRotation());

		// Heading correction / field rel stuff
		ChassisSpeeds speeds = fieldRelative ? fieldRelativeSpeeds : robotRelativeSpeeds;
		boolean headingCorrection = useHeadingCorrection && omegaSupplier.getAsDouble() == 0
				&& (ySupplier.getAsDouble() > HEADING_CORRECTION_DEADBAND || xSupplier.getAsDouble() > HEADING_CORRECTION_DEADBAND);

		// TODO: TEST TEST TEST, this is trash code
		if (headingCorrection) {
			speeds.omegaRadiansPerSecond = headingCorrectionController.calculate(getRobotRotation().getRadians(),
					lastHeading.getRadians());
		} else {
			lastHeading = getRobotRotation();
		}

		runVelocity(speeds);
	}

	/**
	 * Configures the PID controllers used for angle PID
	 * 
	 * @param kP
	 * @param kI
	 * @param kD
	 */
	public void configureAnglePID(double kP, double kI, double kD) {
		for (var module : modules) {
			module.configureTurnPID(kP, kI, kD);
		}
	}

	/**
	 * Configures the feedforward controllers used for drive
	 * Default is pre-calculated and should work
	 * 
	 * @param kV
	 * @param kA
	 */
	public void configureDriveFF(double kS, double kV, double kA) {
		for (var module : modules) {
			module.configureDriveFF(kS, kV, kA);
		}
	}

	/**
	 * Configures the feedforward controllers used for drive
	 * 
	 * @param kP
	 * @param kI
	 * @param kD
	 */
	public void configureDrivePID(double kP, double kI, double kD) {
		for (var module : modules) {
			module.configureDrivePID(kP, kI, kD);
		}
	}

	/**
	 * Updates odometry measurments
	 */
	public void periodic() {
		// Largely taken from Akit example
		odometryLock.lock(); // Prevents odometry updates while reading data

		gyroIO.updateInputs(gyroInputs);
		for (var module : modules) {
			// System.out.println("updating module");
			module.updateInputs();
		}
		odometryLock.unlock();
		Logger.processInputs("Drive/Gyro", gyroInputs);
		for (var module : modules) {
			// Updates odo and runs setpoints
			module.periodic();
		}

		// Stop moving when disabled
		if (DriverStation.isDisabled()) {
			for (var module : modules) {
				module.stop();
			}
			Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
			Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});

		}

		// Update odometry
		double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
		int sampleCount = sampleTimestamps.length;
		for (int i = 0; i < sampleCount; i++) {
			// Read wheel positions and deltas from each module
			SwerveModulePosition[] modulePositions = new SwerveModulePosition[numModules];
			SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[numModules];
			for (int moduleIndex = 0; moduleIndex < numModules; moduleIndex++) {
				modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
				moduleDeltas[moduleIndex] = new SwerveModulePosition(
						modulePositions[moduleIndex].distanceMeters -
								lastModulePositions[moduleIndex].distanceMeters,
						modulePositions[moduleIndex].angle);
				lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
			}

			// Update gyro angle
			if (gyroInputs.connected && !isSim) {
				// Use the real gyro angle (Or the bogus one for sim)
				rawGyroRotation = gyroInputs.odometryYawPositions[i];
			} else {
				// Use the angle delta from the kinematics and module deltas
				Twist2d twist = kinematics.toTwist2d(moduleDeltas);
				rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
				if (isSim)
					gyroIO.setAngle(new Rotation3d(rawGyroRotation.getCos(), 0, rawGyroRotation.getSin()));
			}

			// Apply update
			poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
		}
	}

	/**
	 * @return The robots chassis speeds
	 */
	public ChassisSpeeds getChassisSpeed() {
		ChassisSpeeds robotChassisSpeed = kinematics.toChassisSpeeds(getModuleStates());
		return robotChassisSpeed;
	}

	/**
	 * @return The robots speed in ft/s
	 */
	@AutoLogOutput(key = "SwerveDrive/Speed")
	public double getSpeed() {
		double robotSpeed = Math.sqrt(
				Math.pow(getChassisSpeed().vxMetersPerSecond, 2) +
						Math.pow(getChassisSpeed().vyMetersPerSecond, 2));
		return Units.metersToFeet(robotSpeed);
	}

	/**
	 * Runs the velocity of the robot
	 * 
	 * @param speeds Chassis speeds to set module states to
	 */
	public void runVelocity(ChassisSpeeds speeds) {
		// Taken largely from akit
		// Turns chassis speeds over a time into like splits that u can discretely set
		// module states to
		// TODO: Tune dt time? Hear sum1 talking about how that improves odo significantly
		ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(
				speeds,
				DT_TIME_SECONDS);

		// Turns chassis speeds into module states and then makes sure they're
		// attainable
		SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(
				setpointStates,
				maxSpeed);

		// Send setpoints to modules
		SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) {
			// Sets module setpoints, periodic actually runs the states
			optimizedSetpointStates[i] = modules[i].runState(setpointStates[i]);
		}

		// Log setpoint states
		Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
		Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
	}

	/**
	 * Returns the module states (turn angles and drive velocities) for all of the
	 * modules.
	 */
	@AutoLogOutput(key = "SwerveStates/Measured")
	private SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[numModules];
		for (int i = 0; i < numModules; i++) {
			states[i] = modules[i].getState();
		}
		return states;
	}

	// TODO: Have a working vision system so this matters

	/**
	 * Adds a vision measurement to the pose estimator.
	 *
	 * @param visionPose The pose of the robot as measured by the vision camera.
	 * @param timestamp  The timestamp of the vision measurement in seconds.
	 */
	public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
		poseEstimator.addVisionMeasurement(visionPose, timestamp);
	}

	/**
	 * Adds a vision measurment to the pose estimator.
	 * 
	 * @param visionPose               The pose of the robot measured by the vision
	 *                                 camera.
	 * @param timestamp                The timestamp of the vision measurment in
	 *                                 seconds.
	 * @param visionMeasurementStdDevs The standard deviations of the vision
	 *                                 measurement.
	 */
	public void addVisionMeasurement(
			Pose2d visionPose,
			double timestamp,
			Matrix<N3, N1> visionMeasurementStdDevs) {
		poseEstimator.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
	}

	/**
	 * @return Array of module translations in the order FL, FR, BL, BR
	 */
	public static Translation2d[] getModuleTranslations() {
		return new Translation2d[] {
				new Translation2d(
						trackWidthX / 2,
						trackWidthY / 2),
				new Translation2d(
						trackWidthX / 2,
						-trackWidthY / 2),
				new Translation2d(
						-trackWidthX / 2,
						trackWidthY / 2),
				new Translation2d(
						-trackWidthX / 2,
						-trackWidthY / 2),
		};
	}

	/**
	 * Configures the PID controller used for heading correction
	 * 
	 * @param kP
	 * @param kI
	 * @param kD
	 */
	public void configureHeadingCorrectionPID(double kP, double kI, double kD) {
		headingCorrectionController.setPID(kP, kI, kD);
	}

	/**
	 * @return The robots current rotation as a rotation2d
	 */
	public Rotation2d getRobotRotation() {
		return poseEstimator.getEstimatedPosition().getRotation();
	}

	/**
	 * @return The robots max speed
	 */
	public double getMaxSpeed() {
		return maxSpeed;
	}

	/**
	 * @return The robots calculated max angular velocity
	 */
	public double getMaxAngularVelocity() {
		return maxSpeed / Math.hypot(
				trackWidthX / 2.0,
				trackWidthY / 2.0);
	}
}
