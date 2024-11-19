package frc.robot.pioneersLib.bumSwerve.SwerveMotor;

import java.util.OptionalDouble;
import java.util.Queue;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.pioneersLib.bumSwerve.OdometryThread;
import frc.robot.pioneersLib.bumSwerve.SwerveDrive;
import frc.robot.pioneersLib.bumSwerve.SwerveModule;


public class SwerveMotorIOSparkMax implements SwerveMotorIO {
    private boolean isDrive;

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> motorPositionQueue;

    private PIDController feedbackController;
    private SimpleMotorFeedforward driveFeedForward;

    //Default config values

    private static final boolean MOTOR_INVERTED = true;
    private static final int SPARK_TIMEOUT_MS = 250;
    private static final int MOTOR_CURRENT_LIMIT = 30;
    private static final double MAX_VOLTS = 12.0;
    private static final int SPARK_MEASUREMENT_PERIOD_MS = 10;
    private static final int SPARK_AVG_DEPTH = 2;
    private static final double SPARK_FRAME_PERIOD = 1000.0 / SwerveModule.ODOMETRY_FREQUENCY;

    private static final double RPS_CONVERSION_FACTOR = 60;
    private static final double ENCODER_CF= 1;
    private static final double RAMP_RATE = 1;

    private double gearRatio = 21.4286;

    /**
     * Creates a SparkMax Swerve Motor object pointing to the specified CAN ID. Also need to specify if the motor is used for turn or for drive
     * @param canID
     * @param isDrive
     * @param gearRatio
     */
    public SwerveMotorIOSparkMax(int canID, double gearRatio) {
        this.gearRatio = gearRatio;

        motor = new CANSparkMax(canID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        encoder.setPosition(0);
        encoder.setPositionConversionFactor(ENCODER_CF);

		motor.restoreFactoryDefaults();
		motor.setCANTimeout(SPARK_TIMEOUT_MS);

		motor.setInverted(MOTOR_INVERTED);
		motor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
		motor.enableVoltageCompensation(MAX_VOLTS);
		motor.setClosedLoopRampRate(RAMP_RATE);

		encoder.setMeasurementPeriod(
			SPARK_MEASUREMENT_PERIOD_MS
		);
		encoder.setAverageDepth(SPARK_AVG_DEPTH);

        //TODO: why is this like this
		motor.setCANTimeout(0);
		motor.setPeriodicFramePeriod(
			PeriodicFrame.kStatus2,
			(int) (SPARK_FRAME_PERIOD)
		);

        burnFlash();

        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        motorPositionQueue = OdometryThread.getInstance().registerSignal(() -> {
            double value = encoder.getPosition();
            if (motor.getLastError() == REVLibError.kOk) {
                return OptionalDouble.of(value);
            } else {
                return OptionalDouble.of(0);
            }
        });

        feedbackController = new PIDController(0, 0, 0);
        driveFeedForward = new SimpleMotorFeedforward(0, 0, 0);
    }

    @Override
    public void updateInputs(SwerveMotorIOInputs inputs) {
        inputs.motorPosition = Rotation2d.fromRotations(encoder.getPosition() / gearRatio);
        inputs.motorVelocityRPS = (encoder.getVelocity() / RPS_CONVERSION_FACTOR) / gearRatio;
        inputs.motorCurrentAmps = new double[] { motor.getOutputCurrent() };

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryMotorPositions = motorPositionQueue.stream().map((Double value) -> Rotation2d.fromRotations(value / gearRatio)).toArray(Rotation2d[]::new);

        if (isDrive) inputs.odometryDriveAccumulatedPosition = motorPositionQueue.stream().mapToDouble((Double value) -> value / gearRatio).toArray();

        timestampQueue.clear();
        motorPositionQueue.clear();
    }

    @Override
    public void updateOutputs(SwerveMotorIOOutputs outputs) {
        outputs.motorAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(encoder.getPosition() / gearRatio);
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity() / RPS_CONVERSION_FACTOR / gearRatio;
    }

    @Override
    public double getPositionError() {
        return feedbackController.getPositionError();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setPosition(double setpoint) {
        if (isDrive) throw new UnsupportedOperationException("Cannot set position on a drive motor");

        setVoltage(feedbackController.calculate(getAngle().getDegrees() / (Math.PI * 2), setpoint));
    }

    @Override
    public StatusCode setEncoderPosition(Rotation2d positionDeg) {
        encoder.setPosition(positionDeg.getRotations() * gearRatio);
        return StatusCode.OK;
    }

    @Override
    public void setVelocity(double speedpoint) {
        if (!isDrive) throw new UnsupportedOperationException("Cannot set velocity on a turn motor");

        double velocityRPS = speedpoint / SwerveDrive.wheelRadius;
        setVoltage(driveFeedForward.calculate(velocityRPS) + feedbackController.calculate(getVelocity(), velocityRPS));
    }

    @Override
    public void setBrakeMode(boolean enable) {
        motor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        feedbackController.setPID(kP, kI, kD);
    }

    @Override
    public void configureFF(double kS, double kV, double kA) {
        driveFeedForward = new SimpleMotorFeedforward(0, kV, kA);
    }

    @Override
    public void setIsDrive(boolean isDrive) {
        this.isDrive = isDrive;
    }

    /**
     * Sets the can timeout in ms of the motor. If timeout is reached an error is produced
     * <br></br>
     * Default value is 250 ms
     * @param ms
     */
    public void setCANTimeout(int ms) {
        motor.setCANTimeout(ms);
    }

    /**
     * Sets if the motor is inverted or not. 
     * <br></br>
     * Default value is true
     * @param inverted True is inverted
     */
    public void setInverted(boolean inverted) {
        motor.setInverted(inverted);
    }

    /**
     * Sets the current limit in Amps. The motor controller will reduce the voltage output of the motor to avoid surpassing this limit.
     * <br></br>
     * Default value is 30 Amps
     * 
     * @param limit
     */
    public void setSmartCurrentLimit(int limit) {
        motor.setSmartCurrentLimit(limit);
    }

    /**
     * Sets the voltage compensation setting for all modes on the SPARK and enables voltage compensation
     * <br></br>
     * Default value is 12 volts
     * @param maxVolts
     */
    public void enableVoltageCompensation(int maxVolts) {
        motor.enableVoltageCompensation(maxVolts);
    }

    /**
     * Sets the position measurement period the encoder uses to calculate velocity (equation is position / change in time)
     * <br></br>
     * Default value is 10 ms
     * @param period Period in ms
     */
    public void setMeasurementPeriod(int period) {
        encoder.setMeasurementPeriod(period);
    }

    /**
     * Sets the rate of transmission for periodic frames from the SPARK
     * <br></br>
     * Default value is 4 ms
     * @param period Period in ms
     */
    public void setPeriodicFramePeriod(double period) {
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int) period);
    }

    /**
     * Sets the number of samples in the average for velocity readings
     * <br></br>
     * Default value is 2
     * @param depth
     */
    public void setAverageDepth(int depth) {
        encoder.setAverageDepth(depth);
    }

    /**
     * Writes all settings to flash
     */
    public void burnFlash() {
        motor.burnFlash();
    }
}
