package frc.robot.pioneersLib.bumSwerve;

import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIO;

public class SwerveModule {
    public static final double ODOMETRY_FREQUENCY = 250.0;

    private SwerveMotorIO driveMotor;
    private SwerveMotorIO turnMotor;

    public SwerveModule(SwerveMotorIO driveMotor, SwerveMotorIO turnMotor) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
    }

    public SwerveMotorIO getDriveMotor() {
        return driveMotor;
    }
    
    public SwerveMotorIO getTurnMotor() {
        return turnMotor;
    }
}
