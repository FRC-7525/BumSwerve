package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.pioneersLib.bumSwerve.SwerveDrive;
import frc.robot.pioneersLib.bumSwerve.SwerveModule;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIO;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIONavX;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIOSim;
import frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder.SwerveAbsoluteEncoderIO;
import frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder.SwerveAbsoluteEncoderIOCANcoder;
import frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder.SwerveAbsoluteEncoderIOSim;
import frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder.SwerveAbsoluteEncoderIO.SwerveAbsoluteEncoderIOInputs;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIOKrakenSim;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIONeoSim;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIOSparkMax;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIOTalonFX;
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

    private boolean sim;

    public Drive() {
        super("Drive", DriveStates.REGULAR);
        controller = new XboxController(0);
        sim = true;

        if (sim) {
            gyroIO = new SwerveGyroIOSim();
            // TODO: Finish abs encoder things
            modules = new SwerveModule[] {
                    new SwerveModule(new SwerveMotorIOKrakenSim(1, 5.357, 0.000520786),
                            new SwerveMotorIONeoSim(5, 21.4286),
                            new SwerveAbsoluteEncoderIOSim(9),
                            0, "FrontLeft"),
                    new SwerveModule(new SwerveMotorIOKrakenSim(2, 5.357, 0.000520786),
                            new SwerveMotorIONeoSim(6, 21.4286),
                            new SwerveAbsoluteEncoderIOSim(10),
                            0, "FrontRight"),
                    new SwerveModule(new SwerveMotorIOKrakenSim(3, 5.357, 0.000520786),
                            new SwerveMotorIONeoSim(7, 21.4286),
                            new SwerveAbsoluteEncoderIOSim(11),
                            0, "BackLeft"),
                    new SwerveModule(new SwerveMotorIOKrakenSim(4, 5.357, 0.000520786),
                            new SwerveMotorIONeoSim(8, 21.4286),
                            new SwerveAbsoluteEncoderIOSim(12),
                            0, "BackRight")
            };
        } else {
            gyroIO = new SwerveGyroIONavX(1);
            modules = new SwerveModule[] {
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(1, 5.357),
                            new SwerveMotorIOSparkMax(5, 21.4286),
                            new SwerveAbsoluteEncoderIOCANcoder(2),
                            0, 
                            "FrontLeft"
                    ),
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(2, 5.357),
                            new SwerveMotorIOSparkMax(6, 21.4286),
                            new SwerveAbsoluteEncoderIOCANcoder(2),
                            0, 
                            "FrontRight"
                    ),
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(3, 5.357),
                            new SwerveMotorIOSparkMax(7, 21.4286),
                            new SwerveAbsoluteEncoderIOCANcoder(2),
                            0, 
                            "BackLeft"
                    ),
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(4, 5.357),
                            new SwerveMotorIOSparkMax(8, 21.4286),
                            new SwerveAbsoluteEncoderIOCANcoder(2),
                            0, 
                            "BackRight"
                    )
            };
        }

        drive = new SwerveDrive(TRACK_WIDTH_X, TRACK_WIDTH_Y, modules, gyroIO, MAX_SPEED, WHEEL_RADIUS, sim);
        // TODO: Tune
        drive.configureAnglePID(0.0, 0, 0.0);
        drive.configureDrivePID(0.01, 0, 0);
    }

    public void runState() {
        drive.periodic();

        // Drive the robot
        drive.drive(() -> controller.getLeftY(),  () -> controller.getLeftX(), () -> controller.getRightX(), true, false);
    }
}
