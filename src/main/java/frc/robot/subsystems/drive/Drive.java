package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.pioneersLib.bumSwerve.SwerveDrive;
import frc.robot.pioneersLib.bumSwerve.SwerveModule;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIO;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIONavX;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIOSim;
import frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder.SwerveAbsoluteEncoderIOCANcoder;
import frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder.SwerveAbsoluteEncoderIOSim;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIOKrakenSim;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIONeoSim;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIOSparkMax;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIOTalonFX;
import frc.robot.pioneersLib.subsystem.Subsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;

public class Drive extends Subsystem<DriveStates> {
    private SwerveDrive drive;
    private Vision vision;

    private final double WHEEL_RADIUS = Units.inchesToMeters(2);
    private final double TRACK_WIDTH_X = Units.inchesToMeters(25);
    private final double TRACK_WIDTH_Y = Units.inchesToMeters(25);
    private final double MAX_SPEED = Units.feetToMeters(19.5);

    private SwerveGyroIO gyroIO;
    private SwerveModule[] modules;
    private XboxController controller;

    private boolean sim;
    
    // TODO: Make a Manger
    public Drive() {
        super("Drive", DriveStates.REGULAR);
        controller = new XboxController(0);
        sim = true;

        if (sim) {
            vision = new Vision(new VisionIOSim(), this);
            gyroIO = new SwerveGyroIOSim();
            modules = new SwerveModule[] {
                    new SwerveModule(new SwerveMotorIOKrakenSim(1, 5.357, 0.000520786),
                            new SwerveMotorIONeoSim(5, 21.4286),
                            new SwerveAbsoluteEncoderIOSim(9, 121.0), "FrontLeft"),
                    new SwerveModule(new SwerveMotorIOKrakenSim(2, 5.357, 0.000520786),
                            new SwerveMotorIONeoSim(6, 21.4286),
                            new SwerveAbsoluteEncoderIOSim(10, 11.0), "FrontRight"),
                    new SwerveModule(new SwerveMotorIOKrakenSim(3, 5.357, 0.000520786),
                            new SwerveMotorIONeoSim(7, 21.4286),
                            new SwerveAbsoluteEncoderIOSim(11, 21.0), "BackLeft"),
                    new SwerveModule(new SwerveMotorIOKrakenSim(4, 5.357, 0.000520786),
                            new SwerveMotorIONeoSim(8, 21.4286),
                            new SwerveAbsoluteEncoderIOSim(12, 1), "BackRight")
            };
        } else {
            vision = new Vision(new VisionIOReal(), this);
            gyroIO = new SwerveGyroIONavX(1);
            modules = new SwerveModule[] {
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(1, 5.357),
                            new SwerveMotorIOSparkMax(5, 21.4286),
                            new SwerveAbsoluteEncoderIOCANcoder(2, 315), "FrontLeft"),
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(2, 5.357),
                            new SwerveMotorIOSparkMax(6, 21.4286),
                            new SwerveAbsoluteEncoderIOCANcoder(2, 31),
                            "FrontRight"),
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(3, 5.357),
                            new SwerveMotorIOSparkMax(7, 21.4286),
                            new SwerveAbsoluteEncoderIOCANcoder(2, 315),
                            "BackLeft"),
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(4, 5.357),
                            new SwerveMotorIOSparkMax(8, 21.4286),
                            new SwerveAbsoluteEncoderIOCANcoder(2, 124),
                            "BackRight")
            };
        }

        drive = new SwerveDrive(TRACK_WIDTH_X, TRACK_WIDTH_Y, modules, gyroIO, MAX_SPEED, WHEEL_RADIUS, sim);
        // TODO: Tune
        drive.configureAnglePID(0.1, 0, 0.0);
        drive.configureDrivePID(0.01, 0, 0);
    }

    public void runState() {
        drive.periodic();
        vision.periodic();

        // Drive the robot
        drive.drive(() -> controller.getLeftY(), () -> controller.getLeftX(), () -> controller.getRightX(),
                true, false);
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
