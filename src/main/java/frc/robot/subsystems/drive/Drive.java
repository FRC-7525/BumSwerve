package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.pioneersLib.bumSwerve.SwerveDrive;
import frc.robot.pioneersLib.bumSwerve.SwerveModule;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIO;
import java.util.function.DoubleSupplier;

import frc.robot.pioneersLib.subsystem.Subsystem;

import static frc.robot.Constants.Drive.*;

public class Drive extends Subsystem<DriveStates> {
    private SwerveDrive drive;
    
    public Drive(SwerveModule[] modules, SwerveGyroIO gyroIO, boolean sim) {
        super("Drive", DriveStates.REGULAR);
        // Sim is passed in because I don't want to make two switch statements (think about it)
        drive = new SwerveDrive(TRACK_WIDTH_X, TRACK_WIDTH_Y, modules, gyroIO, MAX_SPEED, WHEEL_RADIUS, sim);

        switch (Constants.ROBOT_STATE) {
            case REAL:
                drive.configureAnglePID(Real.AZIMUTH_PID.kP, Real.AZIMUTH_PID.kI, Real.AZIMUTH_PID.kD);
                drive.configureDrivePID(Real.DRIVE_PID.kP, Real.AZIMUTH_PID.kI, Real.AZIMUTH_PID.kD);
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
        drive.drive(x, y, rot, false, false);
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

    public void addVisionMeasurment(Pose2d visionPose,
            double timestamp,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        drive.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
    }
}
