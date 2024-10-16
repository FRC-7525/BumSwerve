package frc.robot;

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


    public static final class Drive {
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
