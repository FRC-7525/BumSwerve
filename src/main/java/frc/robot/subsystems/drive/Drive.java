package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.Drive.Real;
import frc.robot.Constants.Drive.Sim;
import frc.robot.pioneersLib.bumSwerve.SwerveDrive;
import frc.robot.pioneersLib.bumSwerve.SwerveModule;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIO;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.pioneersLib.subsystem.Subsystem;

import static frc.robot.Constants.Drive.*;

public class Drive extends Subsystem<DriveStates> {
    private SwerveDrive drive;
    
    public Drive(SwerveModule[] modules, SwerveGyroIO gyroIO, boolean sim) {
        super("Drive", DriveStates.REGULAR);
        // Sim is passed in because I don't want to make two switch statements (think about it)
        drive = new SwerveDrive(TRACK_WIDTH_X, TRACK_WIDTH_Y, modules, gyroIO, MAX_SPEED, WHEEL_RADIUS, sim, CONTROLLER_DEADBAND);
        pathPlannerInit();

        switch (Constants.ROBOT_STATE) {
            case REAL:
                drive.configureAnglePID(Real.AZIMUTH_PID.kP, Real.AZIMUTH_PID.kI, Real.AZIMUTH_PID.kD);
                drive.configureDrivePID(Real.DRIVE_PID.kP, Real.DRIVE_PID.kI, Real.DRIVE_PID.kD);
                break;
            case SIM:
                drive.configureAnglePID(Sim.AZIMUTH_PID.kP, Sim.AZIMUTH_PID.kI, Sim.AZIMUTH_PID.kD);
                drive.configureDrivePID(Sim.DRIVE_PID.kP, Sim.DRIVE_PID.kI, Sim.DRIVE_PID.kD);
                break;
            case REPLAY:
                drive.configureAnglePID(0, 0, 0);
                drive.configureDrivePID(0, 0, 0);
                break;
            default:
                break;
        }    
    }

    /**
     * Drive the robot
     * @param x The x supplier for speed (0-1)
     * @param y The y supplier for speed (0-1)
     * @param rot The rotation supplier for speed (0-1)
     * @param fieldRelative Whether the robot should drive field relative
     * @param headingCorrection Whether the robot should correct its heading
     */
    public void drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, boolean fieldRelative,
            boolean headingCorrection) {
        drive.drive(x, y, rot, fieldRelative, false);
    }

    /**
     * Run drive.period, should be called in robot.periodic
     */
    @Override
    public void runState() {
        drive.periodic();
    }

    public Pose2d getPose() {
        return drive.getRobotPose();
    }

    public void setPose(Pose2d pose) {
        drive.setRobotPose(pose);
    }

    public ChassisSpeeds getChassisSpeed() {
        return drive.getChassisSpeed();
    }

    public void runVelocity(ChassisSpeeds chassisSpeeds) {
        drive.runVelocity(chassisSpeeds);
    }

    public void addVisionMeasurment(Pose2d visionPose,
            double timestamp,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        drive.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
    }

    /**
     * Initializes a Holonomic path planner for use in autos
     */
    public void pathPlannerInit() {
		AutoBuilder.configureHolonomic(
			this::getPose, // Robot pose supplier
			this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
			this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
			this::runVelocity, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            //TODO: Bro why cant i input the PIDConstants into the Config
			new HolonomicPathFollowerConfig(
				Constants.Drive.Auto.MAX_MODULE_SPEED, // Max module speed, in m/s
				Constants.Drive.DRIVE_BASE_RADIUS, // Drive base radius in meters. Distance from robot center to
				// furthest module.
				// Replans path if vision or odo detects errors (S tier)
				new ReplanningConfig(true, true) // Default path replanning config. See the API for the options
				// here
            ),
			() -> {
				// Boolean supplier that controls when the path will be mirrored for the red alliance
				// This will flip the path being followed to the red side of the field.
				// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

				var alliance = DriverStation.getAlliance();
				if (alliance.isPresent()) {
					return alliance.get() == DriverStation.Alliance.Red;
				}
				return false;
			},
			this // Reference to this subsystem to set requirements
		);
    }
}
