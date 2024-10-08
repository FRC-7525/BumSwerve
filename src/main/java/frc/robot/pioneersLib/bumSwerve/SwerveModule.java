package frc.robot.pioneersLib.bumSwerve;

import frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder.SwerveAbsoluteEncoderIO;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIO;

public class SwerveModule {
    public static final double ODOMETRY_FREQUENCY = 250.0;

    private SwerveMotorIO driveMotor;
    private SwerveMotorIO turnMotor;
    private SwerveAbsoluteEncoderIO absoluteEncoder;

    public SwerveModule(SwerveMotorIO driveMotor, SwerveMotorIO turnMotor, SwerveAbsoluteEncoderIO absoluteEncoder) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.absoluteEncoder = absoluteEncoder;
    }

    public SwerveMotorIO getDriveMotor() {
        return driveMotor;
    }
    
    public SwerveMotorIO getTurnMotor() {
        return turnMotor;
    }

    public SwerveAbsoluteEncoderIO getEncoder() {
        return absoluteEncoder;
    }
}
