package frc.robot.pioneersLib.bumSwerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder.SwerveAbsoluteEncoderIO;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIO;

public class SwerveModule {

    // TODO: DO AKIT LOGGING IN HERE AND REMOVE MAGIC NUMBERS!
    // TODO: ADD OPEN LOOP CONTROL
    public static final double ODOMETRY_FREQUENCY = 250.0;

    private SwerveMotorIO driveMotor;
    private SwerveMotorIO turnMotor;
    private SwerveAbsoluteEncoderIO absoluteEncoder;
    private Rotation2d turnRelativeEncoderOffset;

    private SwerveModulePosition[] odometryPositions;

    public SwerveModule(SwerveMotorIO driveMotor, SwerveMotorIO turnMotor, SwerveAbsoluteEncoderIO absoluteEncoder) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.absoluteEncoder = absoluteEncoder;
    }

    /**
     * Runs the module at the specified state
     * @param state
     */
    public void runState(SwerveModuleState state) {

        // Finds encoder offset that's used for odo calculations
        if (turnRelativeEncoderOffset == null) {
            turnRelativeEncoderOffset =  Rotation2d.fromDegrees(absoluteEncoder.getRotationDeg()).minus(turnMotor.getAngle());
        }

        // Prevents the turn motor from doing uneeded rotations
        var optimizedState = SwerveModuleState.optimize(state, turnMotor.getAngle());

        // TODO: I'm bad at math, is this right?
        turnMotor.setPosition(optimizedState.angle.getDegrees());
        
        // Converts m/s peeds to rot/s for setpoint then accounts for turn error
        double driveSetpoint = optimizedState.speedMetersPerSecond/ (Units.inchesToMeters(2) * Math.PI * 2);
        driveMotor.setVelocity(Math.cos(turnMotor.getPositionError()) * driveSetpoint);

        // Taken ish from akit advanced example (I'm not rewriting allat)
        // Updates odo so it can be used in pose estimator in SwerveDrive
        int sampleCount = driveMotor.getOdometryTimestamps().length; // All signals are sampled together with the thread so lengths should be the same??
		odometryPositions = new SwerveModulePosition[sampleCount];
		for (int i = 0; i < sampleCount; i++) {
			double positionMeters = driveMotor.getOdometryAccumulatedPositions()[i] * 2 * Math.PI;
			Rotation2d angle =
				turnMotor.getOdometryPositions()[i].plus(
						turnRelativeEncoderOffset != null ? turnRelativeEncoderOffset : new Rotation2d()
					);
			odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
		}
    }
    

    /**
     * Stops the module from moving
     */
    public void stop() {
        driveMotor.setVoltage(0);
        turnMotor.setVoltage(0);
    }

    /**
     * @return the rotation of the wheel with 0 being your absolute encoder's 0
     */
    public Rotation2d getAngle() {
        return turnMotor.getAngle().plus(turnRelativeEncoderOffset);
    }

    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public double[] getOdometryTimestamps() {
        return driveMotor.getOdometryTimestamps();
    }

    // To interface directly with motors/encoders if ur weird

    /**
     * @return The drive motor
     */
    public SwerveMotorIO getDriveMotor() {
        return driveMotor;
    }
    
    /**
     * @return The turn motor
     */
    public SwerveMotorIO getTurnMotor() {
        return turnMotor;
    }

    /**
     * @return The absolute encoder
     */
    public SwerveAbsoluteEncoderIO getEncoder() {
        return absoluteEncoder;
    }
}
