package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.pioneersLib.bumSwerve.SwerveDrive;
import frc.robot.pioneersLib.bumSwerve.SwerveModule;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIO;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIONavX;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIOSim;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIOKrakenSim;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIONeoSim;
import frc.robot.pioneersLib.subsystem.Subsystem;

public class Drive extends Subsystem<DriveStates> {
    private SwerveDrive drive;

    private final double WHEEL_RADIUS = Units.inchesToMeters(2);
    private final double TRACK_WIDTH_X = Units.inchesToMeters(25);
    private final double TRACK_WIDTH_Y = Units.inchesToMeters(25);
    private final double MAX_SPEED = Units.feetToMeters(19.5);

    private SwerveGyroIO gyroIO;
    private SwerveModule[] modules;
    private XboxController controller;

    Drive() {
        super("Drive", DriveStates.REGULAR);
        controller = new XboxController(0);
        gyroIO = new SwerveGyroIOSim();
        // TODO: Finish abs encoder things
        modules = new SwerveModule[] {
                new SwerveModule(new SwerveMotorIOKrakenSim(1, true, 3, 3), new SwerveMotorIONeoSim(5, false, 3), null,
                        0, "FrontLeft"),
                new SwerveModule(new SwerveMotorIOKrakenSim(2, true, 3, 3), new SwerveMotorIONeoSim(5, false, 3), null,
                        0, "FrontRight"),
                new SwerveModule(new SwerveMotorIOKrakenSim(3, true, 3, 3), new SwerveMotorIONeoSim(5, false, 3), null,
                        0, "BackLeft"),
                new SwerveModule(new SwerveMotorIOKrakenSim(4, true, 3, 3), new SwerveMotorIONeoSim(5, false, 3), null,
                        0, "BackRight")
        };

        drive = new SwerveDrive(TRACK_WIDTH_X, TRACK_WIDTH_Y, modules, gyroIO, MAX_SPEED, WHEEL_RADIUS, true);
    }

    public void runState() {
        drive.periodic();
        
        // Drive the robot
        drive.drive(controller.getLeftX(), controller.getLeftY(), controller.getRightX(), false, false);
    }
}
