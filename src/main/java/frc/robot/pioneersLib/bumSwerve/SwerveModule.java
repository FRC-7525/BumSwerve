package frc.robot.pioneersLib.bumSwerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder.SwerveAbsoluteEncoderIO;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIO;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIOInputsAutoLogged;

public class SwerveModule {

    // TODO: DO AKIT LOGGING IN HERE AND REMOVE MAGIC NUMBERS!
    // TODO: ADD OPEN LOOP CONTROL
    public static final double ODOMETRY_FREQUENCY = 250.0;

    private SwerveMotorIO driveMotor;
    private SwerveMotorIO turnMotor;
    private SwerveAbsoluteEncoderIO absoluteEncoder;
    private Rotation2d turnRelativeEncoderOffset;

    private SwerveModulePosition[] odometryPositions;

    private SwerveMotorIOInputsAutoLogged turnInputs;
    private SwerveMotorIOInputsAutoLogged driveInputs;

    private double absoluteEncoderOffset;
    private String moduleName;

    private Double speedSetPoint;
    private Double angleSetPoint;
    
    /**
     * Creates a new SwerveModule
     * @param driveMotor
     * @param turnMotor
     * @param absoluteEncoder
     * @param absoluteEncoderOffset
     * @param moduleName
     */
    public SwerveModule(SwerveMotorIO driveMotor, SwerveMotorIO turnMotor, SwerveAbsoluteEncoderIO absoluteEncoder, double absoluteEncoderOffset, String moduleName) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.absoluteEncoder = absoluteEncoder;
        this.moduleName = moduleName;
        this.absoluteEncoderOffset = absoluteEncoderOffset;

        this.turnInputs = new SwerveMotorIOInputsAutoLogged();
        this.driveInputs = new SwerveMotorIOInputsAutoLogged();
    }

    // TODO: Make a "Set is drive" function in the motorIO so you don't have to feed in "In drive" when you create a swerve motor

    /**
     * Sets a feed forward controller based on several real-world parameters.
     * <br></br>
     * Controller is used only for drive
     * @param optimalVoltage
     * @param maxLinearSpeed In meters
     * @param wheelGripCoefficientOfFriction
     */
    public SimpleMotorFeedforward setFeedForward(double optimalVoltage, double maxLinearSpeed, double wheelGripCoefficientOfFriction) {
        double kv = optimalVoltage / maxLinearSpeed;
        // ^ Volt-seconds per meter (max voltage divided by max speed)
        double ka = optimalVoltage / calculateMaxAcceleration(wheelGripCoefficientOfFriction);
        // ^ Volt-seconds^2 per meter (max voltage divided by max accel)
        return new SimpleMotorFeedforward(0, kv, ka);
    }

    /**
     * Calculates the max acceleration of the wheel given the coefficient of friction and using gravity
     * @param cof Coefficient of friction
     * @return Max acceleration of the wheel
     */
	public double calculateMaxAcceleration(double cof) {
		return cof * 9.81;
	}

    /**
     * Runs the module at the specified state
     * @param state
     */
    public SwerveModuleState runState(SwerveModuleState state) {

        // Finds encoder offset that's used for odo calculations
        if (turnRelativeEncoderOffset == null) {
            turnRelativeEncoderOffset =  Rotation2d.fromDegrees(absoluteEncoder.getRotationDeg()).minus(turnMotor.getAngle());
        }

        // Prevents the turn motor from doing uneeded rotations
        var optimizedState = SwerveModuleState.optimize(state, turnMotor.getAngle());

        // TODO: I'm bad at math, is this right?
        angleSetPoint = optimizedState.angle.getDegrees();

        speedSetPoint = Math.cos(Units.rotationsToRadians(turnMotor.getPositionError())) * (optimizedState.speedMetersPerSecond/ (Units.inchesToMeters(2) * Math.PI * 2));

        return optimizedState;
    }

    /**
     * Runs the module at the specified state
     */
    public void periodic() {
        if (angleSetPoint != null) {
            turnMotor.setPosition(angleSetPoint);

            if (speedSetPoint != null) {
                driveMotor.setVelocity(speedSetPoint);
            }
        }

        // Taken ish from akit advanced example (I'm not rewriting allat)
        // Updates odo so it can be used in pose estimator in SwerveDrive
        int sampleCount = driveInputs.odometryTimestamps.length; // All signals are sampled together with the thread so lengths should be the same??
		odometryPositions = new SwerveModulePosition[sampleCount];
		for (int i = 0; i < sampleCount; i++) {
			double positionMeters = driveInputs.odometryMotorAccumulatedPosition[i] * 2 * Math.PI;
			Rotation2d angle =
				turnInputs.odometryMotorPositions[i].plus(
						turnRelativeEncoderOffset != null ? turnRelativeEncoderOffset : new Rotation2d()
					);
			odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
		}
    }
    
    public void updateInputs() {
        turnMotor.updateInputs(turnInputs);
        driveMotor.updateInputs(driveInputs);
        Logger.processInputs("Drive/" + moduleName +"/DriveMotor", driveInputs);
        Logger.processInputs("Drive/" + moduleName + "/TurnMotor", turnInputs);
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
        return turnRelativeEncoderOffset != null ?turnMotor.getAngle().plus(turnRelativeEncoderOffset) : turnMotor.getAngle();
    }

    /**
     * @return The calculated odometry positions of each module
     */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /**
     * Configures the PID of the turn motor
     * @param kP
     * @param kI
     * @param kD
     */
    public void configureTurnPID(double kP, double kI, double kD) {
        turnMotor.configurePID(kP, kI, kD);
    }

    /**
     * Configures the FF of the drive motor
     * @param kV
     * @param kA
     */
    public void configureDriveFF(double kS, double kV, double kA) {
        driveMotor.configureFF(kS, kV, kA);
    }

    /**
     * Configures the PID of the drive motor
     * @param kP
     * @param kI
     * @param kD
     */
    public void configureDrivePID(double kP, double kI, double kD) {
        driveMotor.configurePID(kP, kI, kD);
    }


    /**
     * Note: this only returns timestamps from the drive motor
     * which should be the same as the turn motor because the thread takes samples at the same time
     * @return The timestamp of the odometry data
     */
    public double[] getOdometryTimestamps() {
        return driveInputs.odometryTimestamps;
    }

    /**
     * @return Module speed in m/s
     */
    public double getSpeed() {
        return driveInputs.motorVelocityRPS * Units.inchesToMeters(2) * Math.PI * 2;
    }

    /**
     * @return The swerve module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), getAngle());
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
