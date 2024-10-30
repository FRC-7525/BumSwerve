package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.pioneersLib.bumSwerve.SwerveModule;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIO;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIONavX;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIOSim;
import frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder.SwerveAbsoluteEncoderIOCANcoder;
import frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder.SwerveAbsoluteEncoderIOSim;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIOTalonFXSim;
import frc.robot.pioneersLib.controlConstants.PIDConstants;
import frc.robot.pioneersLib.misc.VisionUtil.CameraResolution;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIONeoSim;
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

        public static final double CAMERA_DEBOUNCE_TIME = 0.5;

        // TODO: What camera resolutions actually are these? Assuming they're high bc 1080p is high
        public static final CameraResolution SIDE_RESOLUTION = CameraResolution.HIGH_RES;
        public static final CameraResolution FRONT_RESOLUTION = CameraResolution.HIGH_RES;

        // Other
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    }

    public static final class Drive {

        public enum DriveBase {
            // TODO: Get actual values on a lot of these lol (everything 0.001 is unknown)
            KRAKEN_SWERVE(5.357, 0.001, DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1)),
            NEO_SWERVE(0.001, 21.4286, DCMotor.getNEO(1), DCMotor.getNEO(1)),
            FALCON_AZIMUTH_KRAKEN_DRIVE(5.357, 21.4286, DCMotor.getKrakenX60(1), DCMotor.getFalcon500(1)),
            NEO_AZIMUTH_KRAKEN_DRIVE(5.357, 21.4286, DCMotor.getKrakenX60(1), DCMotor.getNEO(1)),;

            /**
             * Creates a DriveBase with the specified gearing and simulated motors
             * @param driveGearing
             * @param azimuthGearing
             * @param driveMotorSim
             * @param azimuthMotorSim
             */
            DriveBase(double driveGearing, double azimuthGearing, DCMotor driveMotorSim, DCMotor azimuthMotorSim) {
                this.driveGearing = driveGearing;
                this.azimuthGearing = azimuthGearing;
                this.driveMotorSim = driveMotorSim;
                this.azimuthMotorSim = azimuthMotorSim;
            }

            public DCMotor createDriveSim() {
                switch (this) {
                    case KRAKEN_SWERVE:
                        return DCMotor.getKrakenX60(1);
                    case NEO_SWERVE:
                        return DCMotor.getNEO(1);
                    case FALCON_AZIMUTH_KRAKEN_DRIVE:
                        return DCMotor.getKrakenX60(1);
                    case NEO_AZIMUTH_KRAKEN_DRIVE:
                        return DCMotor.getKrakenX60(1);
                    default:
                        throw new IllegalArgumentException("Unknown drive base: " + this);
                }
            }

            public DCMotor createAzimuthSim() {
                switch (this) {
                    case KRAKEN_SWERVE:
                        return DCMotor.getKrakenX60(1);
                    case NEO_SWERVE:
                        return DCMotor.getNEO(1);
                    case FALCON_AZIMUTH_KRAKEN_DRIVE:
                        return DCMotor.getFalcon500(1);
                    case NEO_AZIMUTH_KRAKEN_DRIVE:
                        return DCMotor.getNEO(1);
                    default:
                        throw new IllegalArgumentException("Unknown drive base: " + this);
                }
            }

            public double driveGearing;
            public double azimuthGearing;
            public DCMotor azimuthMotorSim;
            public DCMotor driveMotorSim;
        }

        public static final DriveBase DRIVE_BASE = DriveBase.FALCON_AZIMUTH_KRAKEN_DRIVE;

        // Canbus to use
        public static final boolean CANIVORE_USED = false;
        public static final String RIO_BUS = "rio";
        // TODO: Change to whatever we name our canivore bus
        public static final String CANIVORE_BUS = "canivore";
        private static final String CAN_BUS = CANIVORE_USED ? CANIVORE_BUS : RIO_BUS;

        // Deabands
        public static final double CONTROLLER_DEADBAND = 0.05;

        // Physical Measurments
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
        public static final double TRACK_WIDTH_X = Units.inchesToMeters(25);
        public static final double TRACK_WIDTH_Y = Units.inchesToMeters(25);
        public static final double MAX_SPEED = Units.feetToMeters(19.5);
        public static final class Sim {
            public static final SwerveGyroIO GYRO_IO = new SwerveGyroIOSim();
            public static final PIDConstants DRIVE_PID = new PIDConstants(0.04, 0, 0);
            public static final PIDConstants AZIMUTH_PID = new PIDConstants(50, 0, 0);

            // MOIs, should be neglidgeble and in turn the same for all motors
            public static final double DRIVE_MOI = 0.000520786;
            public static final double AZIMUTH_MOI = 0.000520786;

            public static final SwerveModule[] MODULE_IO = new SwerveModule[] {
                    new SwerveModule(new SwerveMotorIOTalonFXSim(1, DRIVE_BASE.driveGearing, DRIVE_MOI, DRIVE_BASE.createDriveSim()),
                            new SwerveMotorIOTalonFXSim(5, DRIVE_BASE.azimuthGearing, AZIMUTH_MOI, DRIVE_BASE.createAzimuthSim()),
                            new SwerveAbsoluteEncoderIOSim(9, 121.0), "FrontLeft"),
                    new SwerveModule(new SwerveMotorIOTalonFXSim(50, DRIVE_BASE.driveGearing, DRIVE_MOI, DRIVE_BASE.createDriveSim()),
                            new SwerveMotorIOTalonFXSim(6, DRIVE_BASE.azimuthGearing, AZIMUTH_MOI,  DRIVE_BASE.createAzimuthSim()),
                            new SwerveAbsoluteEncoderIOSim(10, 11.0), "FrontRight"),
                    new SwerveModule(new SwerveMotorIOTalonFXSim(3, DRIVE_BASE.driveGearing, DRIVE_MOI, DRIVE_BASE.createDriveSim()),
                            new SwerveMotorIOTalonFXSim(7, DRIVE_BASE.azimuthGearing, AZIMUTH_MOI, DRIVE_BASE.createAzimuthSim()),
                            new SwerveAbsoluteEncoderIOSim(11, 21.0), "BackLeft"),
                    new SwerveModule(new SwerveMotorIOTalonFXSim(14, DRIVE_BASE.driveGearing, DRIVE_MOI, DRIVE_BASE.createDriveSim()),
                            new SwerveMotorIOTalonFXSim(13, DRIVE_BASE.azimuthGearing, AZIMUTH_MOI, DRIVE_BASE.createAzimuthSim()),
                            new SwerveAbsoluteEncoderIOSim(12, 1), "BackRight")
            };    
            // public static final SwerveModule[] MODULE_IO = new SwerveModule[] {
            //         new SwerveModule(new SwerveMotorIOTalonFXSim(1, DRIVE_BASE.driveGearing, DRIVE_MOI, DRIVE_BASE.createDriveSim()),
            //                 new SwerveMotorIONeoSim(5, DRIVE_BASE.azimuthGearing),
            //                 new SwerveAbsoluteEncoderIOSim(9, 121.0), "FrontLeft"),
            //         new SwerveModule(new SwerveMotorIOTalonFXSim(2, DRIVE_BASE.driveGearing, DRIVE_MOI, DRIVE_BASE.createDriveSim()),
            //                 new SwerveMotorIONeoSim(6, DRIVE_BASE.azimuthGearing),
            //                 new SwerveAbsoluteEncoderIOSim(10, 11.0), "FrontRight"),
            //         new SwerveModule(new SwerveMotorIOTalonFXSim(3, DRIVE_BASE.driveGearing, DRIVE_MOI, DRIVE_BASE.createDriveSim()),
            //                 new SwerveMotorIONeoSim(7, DRIVE_BASE.azimuthGearing),
            //                 new SwerveAbsoluteEncoderIOSim(11, 21.0), "BackLeft"),
            //         new SwerveModule(new SwerveMotorIOTalonFXSim(4, DRIVE_BASE.driveGearing, DRIVE_MOI, DRIVE_BASE.createDriveSim()),
            //                 new SwerveMotorIONeoSim(8, DRIVE_BASE.azimuthGearing),
            //                 new SwerveAbsoluteEncoderIOSim(12, 1), "BackRight")
            // };     
        }

        public static final class Real {
            public static final PIDConstants DRIVE_PID = new PIDConstants(0.001, 0, 0);
            public static final PIDConstants AZIMUTH_PID = new PIDConstants(0.01, 0, 0);
            public static final SwerveGyroIO GYRO_IO = new SwerveGyroIONavX(1);
            public static final SwerveModule[] MODULE_IO = new SwerveModule[] {
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(2, CAN_BUS, DRIVE_BASE.driveGearing),
                            new SwerveMotorIOTalonFX(1, CAN_BUS, DRIVE_BASE.azimuthGearing),
                            new SwerveAbsoluteEncoderIOCANcoder(3, 0), 
                            "FrontLeft"),
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(5, CAN_BUS, DRIVE_BASE.driveGearing),
                            new SwerveMotorIOTalonFX(4, CAN_BUS, DRIVE_BASE.azimuthGearing),
                            new SwerveAbsoluteEncoderIOCANcoder(6, 0),
                            "FrontRight"),
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(11, CAN_BUS, DRIVE_BASE.driveGearing),
                            new SwerveMotorIOTalonFX(10, CAN_BUS, DRIVE_BASE.azimuthGearing),
                            new SwerveAbsoluteEncoderIOCANcoder(12, 0),
                            "BackLeft"),
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(8, CAN_BUS, DRIVE_BASE.driveGearing),
                            new SwerveMotorIOTalonFX(7, CAN_BUS, DRIVE_BASE.azimuthGearing),
                            new SwerveAbsoluteEncoderIOCANcoder(9, 0),
                            "BackRight")
            };
        }
    }    
}
