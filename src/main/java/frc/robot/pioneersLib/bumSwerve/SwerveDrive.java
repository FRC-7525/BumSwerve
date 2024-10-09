package frc.robot.pioneersLib.bumSwerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIO;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIO;

public class SwerveDrive {
    static final Lock odometryLock = new ReentrantLock();
	private final SwerveGyroIOInputsAutoLogged gyroInputs = new SwerveGyroIOInputsAutoLogged();

    private PIDController headingCorrectionController;
    private SwerveGyroIO gyroIO;
    private final SwerveModule[] modules; // FL, FR, BL, BR
	private Rotation2d rawGyroRotation = new Rotation2d();

    // TODO: Take in track width in the constructor
    private static double trackWidthX;
    private static double trackWidthY;

    private double maxSpeed;
    private double wheelRadius;

	private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveModulePosition[] lastModulePositions =
		new SwerveModulePosition[] {
			new SwerveModulePosition(),
			new SwerveModulePosition(),
			new SwerveModulePosition(),
			new SwerveModulePosition(),
		};

    private SwerveModuleState[] setpoints;

    // TODO: Uh i mean u can only have 4 modules rn make that not so
    public SwerveDrive(double trackWidthX, double trackWidthY, SwerveModule[] modules, SwerveGyroIO gyroIO, double maxSpeed, double wheelRadius) {
        SwerveDrive.trackWidthX = trackWidthX;
        SwerveDrive.trackWidthY = trackWidthY;
        OdometryThread.getInstance().start();

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
        headingCorrectionController = new PIDController(0, 0, 0);

        this.gyroIO = gyroIO;
        this.maxSpeed = maxSpeed;
        this.wheelRadius = wheelRadius;

        this.modules = modules;
    }

    public void periodic() {
		odometryLock.lock(); // Prevents odometry updates while reading data
		gyroIO.updateInputs(gyroInputs);
		for (var module : modules) {
			module.updateInputs();
		}
		odometryLock.unlock();
		Logger.processInputs("Drive/Gyro", gyroInputs);
		for (var module : modules) {
			module.runState(new SwerveModuleState());
		}

		// Stop moving when disabled
		if (DriverStation.isDisabled()) {
			for (var module : modules) {
				module.stop();
			}
		}

		// Update odometry
		double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
		int sampleCount = sampleTimestamps.length;
		for (int i = 0; i < sampleCount; i++) {
			// Read wheel positions and deltas from each module
			SwerveModulePosition[] modulePositions =
				new SwerveModulePosition[Constants.Drive.NUM_MODULES];
			SwerveModulePosition[] moduleDeltas =
				new SwerveModulePosition[Constants.Drive.NUM_MODULES];
			for (int moduleIndex = 0; moduleIndex < Constants.Drive.NUM_MODULES; moduleIndex++) {
				modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
				moduleDeltas[moduleIndex] = new SwerveModulePosition(
					modulePositions[moduleIndex].distanceMeters -
					lastModulePositions[moduleIndex].distanceMeters,
					modulePositions[moduleIndex].angle
				);
				lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
			}

			// Update gyro angle
			if (gyroInputs.connected) {
				// Use the real gyro angle
				rawGyroRotation = gyroInputs.odometryYawPositions[i];
			} else {
				// Use the angle delta from the kinematics and module deltas
				Twist2d twist = kinematics.toTwist2d(moduleDeltas);
				rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
			}

			// Apply update
			poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
		}
    }

    // Taken from akit example
    public void runVelocity(ChassisSpeeds speeds) {
        // Turns chassis speeds over a time into like splits that u can discretley set module states to
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(
            speeds,
            0.02
        );
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            setpointStates,
            maxSpeed
        );

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates =
            new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = modules[i].runState(setpointStates[i]);
        }

        // Log setpoint states
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    }

    /**
     * @return Array of module translations in the order FL, FR, BL, BR
     */
    public static Translation2d[] getModuleTranslations() {
		return new Translation2d[] {
			new Translation2d(
				trackWidthX / 2,
				trackWidthY / 2
			),
			new Translation2d(
				trackWidthX / 2,
				-trackWidthY / 2
			),
			new Translation2d(
				-trackWidthX / 2,
				trackWidthY / 2
			),
			new Translation2d(
				-trackWidthX / 2,
				-trackWidthY / 2
			),
		};
	}
}
