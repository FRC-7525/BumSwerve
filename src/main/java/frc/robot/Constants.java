package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.pioneersLib.VisionUtil.CameraResolution;
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

public class Constants {
    public enum RobotState {
        SIM, REAL, REPLAY
    }
  
    public static final RobotState ROBOT_STATE = RobotState.SIM;
    
    public static final XboxController CONTROLLER = new XboxController(0);
    public static final XboxController OPERATOR_CONTROLLER = new XboxController(1);

    public static final class Vision {
        // Robot to cam 
        public static final Translation3d ROBOT_TO_SIDE_CAMERA_TRALSLATION = new Translation3d(Units.inchesToMeters(-7.19), Units.inchesToMeters(11), Units.inchesToMeters(15.25));
        public static final Rotation3d ROBOT_TO_SIDE_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-20), Math.toRadians(90));
        public static final Transform3d ROBOT_TO_SIDE_CAMERA = new Transform3d(ROBOT_TO_SIDE_CAMERA_TRALSLATION, ROBOT_TO_SIDE_CAMERA_ROTATION);
        public static final Translation3d ROBOT_TO_FRONT_CAMERA_TRANSLATION = new Translation3d(Units.inchesToMeters(-14.25), 0, Units.inchesToMeters(6));
        public static final Rotation3d ROBOT_TO_FRONT_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-67), Math.toRadians(180));
        public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(ROBOT_TO_FRONT_CAMERA_TRANSLATION, ROBOT_TO_FRONT_CAMERA_ROTATION);

        // TODO: What camera resolutions actually are these? Assuming they're high bc 1080p is high
        public static final CameraResolution SIDE_RESOLUTION = CameraResolution.HIGH_RES;
        public static final CameraResolution FRONT_RESOLUTION = CameraResolution.HIGH_RES;

        // Other
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    }

    public static final class Drive {

        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
        public static final double TRACK_WIDTH_X = Units.inchesToMeters(25);
        public static final double TRACK_WIDTH_Y = Units.inchesToMeters(25);
        public static final double MAX_SPEED = Units.feetToMeters(19.5);
        public static final class Sim {
            public static final SwerveGyroIO GYRO_IO = new SwerveGyroIOSim();
            public static final SwerveModule[] MODULE_IO = new SwerveModule[] {
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
        }

        public static final class Real {
            public static final SwerveGyroIO GYRO_IO = new SwerveGyroIONavX(1);
            public static final SwerveModule[] MODULE_IO = new SwerveModule[] {
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(2, 5.357),
                            new SwerveMotorIOSparkMax(1, 21.4286),
                            new SwerveAbsoluteEncoderIOCANcoder(3, 171.826171875), 
                            "FrontLeft"),
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(5, 5.357),
                            new SwerveMotorIOSparkMax(4, 21.4286),
                            new SwerveAbsoluteEncoderIOCANcoder(6, -23.115234375),
                            "FrontRight"),
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(11, 5.357),
                            new SwerveMotorIOSparkMax(10, 21.4286),
                            new SwerveAbsoluteEncoderIOCANcoder(12, 220.517578125),
                            "BackLeft"),
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(8, 5.357),
                            new SwerveMotorIOSparkMax(7, 21.4286),
                            new SwerveAbsoluteEncoderIOCANcoder(9, -67.939453125),
                            "BackRight")
            };
        }
    }    
}
