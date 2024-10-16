package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.pioneersLib.bumSwerve.SwerveDrive;
import frc.robot.pioneersLib.bumSwerve.SwerveModule;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIO;
import frc.robot.pioneersLib.subsystem.Subsystem;

public class Drive extends Subsystem<DriveStates> {
    private SwerveDrive drive;

    public Drive(SwerveModule[] modules, SwerveGyroIO gyroIO, boolean sim) {
        super("Drive", DriveStates.REGULAR);
        // Sim is passed in because I don't want to make two switch statements (think about it)
        drive = new SwerveDrive(Constants.Drive.TRACK_WIDTH_X, Constants.Drive.TRACK_WIDTH_Y, modules, gyroIO, Constants.Drive.MAX_SPEED, Constants.Drive.WHEEL_RADIUS, sim);

        switch (Constants.ROBOT_STATE) {
            case REAL:
                drive.configureAnglePID(0.5, 0, 0);
                drive.configureDrivePID(0.01, 0, 0);
                break;
            case SIM:
                drive.configureAnglePID(0.5, 0, 0);
                drive.configureDrivePID(0.01, 0, 0);
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
        drive.drive(x, y, rot, fieldRelative, headingCorrection);
    }

    /**
     * Run drive.period, should be called in robot.periodic
     */
    @Override
    public void runState() {
        drive.periodic();
    }
}
