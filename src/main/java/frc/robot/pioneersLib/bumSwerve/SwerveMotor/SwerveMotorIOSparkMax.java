package frc.robot.pioneersLib.bumSwerve.SwerveMotor;

import java.util.OptionalDouble;
import java.util.Queue;

import org.opencv.features2d.Feature2D;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.google.flatbuffers.Constants;
import com.kauailabs.navx.AHRSProtocol.TuningVar;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.pioneersLib.bumSwerve.OdometryThread;
import frc.robot.pioneersLib.bumSwerve.SwerveModule;


public class SwerveMotorIOSparkMax implements SwerveMotorIO {
    private boolean isDrive;

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> motorPositionQueue;

    private PIDController feedbackController;
    private SimpleMotorFeedforward drivFeedforward;

    //Default config values
    private final double OPTIMAL_VOLTAGE = 12;
    private final double MAX_LINEAR_SPEED = Units.feetToMeters(19.6);
    private final double WHEEL_GRIP_COEFFICIENT_OF_FRICTION = 1.19;

    private final boolean MOTOR_INVERTED = true;
    private final int SPARK_TIMEOUT_MS = 250;
    private final int MOTOR_CURRENT_LIMIT = 30;
    private final double MAX_VOLTS = 12.0;
    private final int SPARK_MEASUREMENT_PERIOD_MS = 10;
    private final int SPARK_AVG_DEPTH = 2;
    private final double SPARK_FRAME_PERIOD = 1000.0 / SwerveModule.ODOMETRY_FREQUENCY;

    private final double RPS_CONVERSION_FACTOR = 60;
    private final double DEGREE_CONVERSION_FACTOR = 360;

    private double MOTOR_GEAR_RATIO = 21.4286;

    /**
     * Creates a SparkMax Swerve Motor object pointing to the specified CAN ID. Also need to specify if the motor is used for turn or for drive
     * @param canID
     * @param isDrive
     */
    public SwerveMotorIOSparkMax(int canID, boolean isDrive, double gearRatio) {
        this.isDrive = isDrive;

        MOTOR_GEAR_RATIO = gearRatio;

        motor = new CANSparkMax(canID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        encoder.setPosition(0);

		motor.restoreFactoryDefaults();
		motor.setCANTimeout(SPARK_TIMEOUT_MS);

		motor.setInverted(MOTOR_INVERTED);
		motor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
		motor.enableVoltageCompensation(MAX_VOLTS);
		motor.setClosedLoopRampRate(1);

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

        // See IO for why it's commented, just use an empty simpleFF and have a configure FF function in the IO that u feed the output of the current SetFF into bc setFF only calculates kV
        // if (isDrive) {
        //     setFeedForward(OPTIMAL_VOLTAGE, MAX_LINEAR_SPEED, WHEEL_GRIP_COEFFICIENT_OF_FRICTION);
        // }
    }

    @Override
    public void updateInputs(SwerveMotorIOInputs inputs) {
        inputs.motorPosition = Rotation2d.fromRotations(encoder.getPosition() / MOTOR_GEAR_RATIO); //TODO: Switch these out with getters
        inputs.motorVelocityRPS = (encoder.getVelocity() / RPS_CONVERSION_FACTOR) / MOTOR_GEAR_RATIO;
        inputs.motorCurrentAmps = new double[] { motor.getOutputCurrent() };

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryMotorPositions = motorPositionQueue.stream().map((Double value) -> Rotation2d.fromRotations(value / MOTOR_GEAR_RATIO)).toArray(Rotation2d[]::new);

        timestampQueue.clear();
        motorPositionQueue.clear();
    }

    @Override
    public void updateOutputs(SwerveMotorIOOutputs outputs) {
        outputs.motorAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setPosition(double setpoint) {
        if (isDrive) throw new Error("Cannot use setPosition with a drive motor");

        //TODO: Create getter for position
        setVoltage(feedbackController.calculate(encoder.getPosition() * DEGREE_CONVERSION_FACTOR, setpoint));
    }

    @Override
    public void setVelocity(double speedpoint) {
        if (!isDrive) throw new Error("Cannot use setVelocity with a turn motor");

        //TODO: Find better way to have constants
        double velocityRPS = speedpoint / 2.0 /*TODO: Switch this out with a constant */;
        //TODO: Create getter for velocity
        setVoltage(drivFeedforward.calculate(velocityRPS) + feedbackController.calculate(encoder.getVelocity() / RPS_CONVERSION_FACTOR / MOTOR_GEAR_RATIO, velocityRPS));
    }

    @Override
    public void setBrakeMode(boolean enable) {
        motor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        feedbackController.setPID(kP, kI, kD);
    }

    // See IO for why it's commented
    // @Override
    // public void setFeedForward(double optimalVoltage, double maxLinearSpeed, double wheelGripCoefficientOfFriction) {  
    //     double kv = optimalVoltage / maxLinearSpeed;
    //     // ^ Volt-seconds per meter (max voltage divided by max speed)
    //     double ka = optimalVoltage / calculateMaxAcceleration(wheelGripCoefficientOfFriction);
    //     // ^ Volt-seconds^2 per meter (max voltage divided by max accel)
    //     drivFeedforward = new SimpleMotorFeedforward(0, kv, ka);
    // }

    @Override
	public double calculateMaxAcceleration(double cof) {
		return cof * 9.81;
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
