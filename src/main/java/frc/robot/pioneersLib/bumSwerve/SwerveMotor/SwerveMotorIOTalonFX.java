package frc.robot.pioneersLib.bumSwerve.SwerveMotor;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.cfg.ConstructorDetector.SingleArgConstructor;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.pioneersLib.bumSwerve.OdometryThread;
import frc.robot.pioneersLib.bumSwerve.SwerveDrive;

public class SwerveMotorIOTalonFX implements SwerveMotorIO {
    private final TalonFX motor;

    private final StatusSignal<Double> motorPosition;
    private final StatusSignal<Double> motorVelocity;
    private final StatusSignal<Double> motorAppliedVolts;
    private final StatusSignal<Double> motorCurrent;

    private PIDController feedbackController;
    private SimpleMotorFeedforward drivFeedforward;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> motorPositionQueue;

    private double gearRatio;
    private final boolean isDrive;

    TalonFXConfiguration driveConfig;

    //Default config values
    private static final double CURRENT_LIMIT = 40.0;
    private static final double POSITION_UPDATE_FREQUENCY = 250.0;
    private static final double SIGNAL_UPDATE_FREQUENCY = 50.0;
    private static final double VOLTAGE_CLOSED_LOOP_RAMP_PERIOD = 0.15;

    public SwerveMotorIOTalonFX(int canID, boolean isDrive, double gearRatio) {
        this.gearRatio = gearRatio;
        this.isDrive = isDrive;

        motor = new TalonFX(canID);

        driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motor.getConfigurator().apply(driveConfig);
        //TODO: Apparently this might not work (idk what it's even supposed to do)
        motor.getConfigurator().apply(driveConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(VOLTAGE_CLOSED_LOOP_RAMP_PERIOD)); 

        motor.optimizeBusUtilization();

        motorPosition = motor.getPosition();
		motorVelocity = motor.getVelocity();
		motorAppliedVolts = motor.getMotorVoltage();
		motorCurrent = motor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(POSITION_UPDATE_FREQUENCY, motorPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
			SIGNAL_UPDATE_FREQUENCY,
			motorVelocity,
			motorAppliedVolts,
			motorCurrent
		);

        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
		motorPositionQueue = OdometryThread.getInstance().registerSignal(motor, motor.getPosition());

        feedbackController = new PIDController(0, 0, 0);
        drivFeedforward = new SimpleMotorFeedforward(0, 0, 0);
    }

    @Override
    public void updateInputs(SwerveMotorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            motorPosition,
            motorVelocity,
            motorAppliedVolts,
            motorCurrent
        );

        inputs.motorVelocityRPS = motorVelocity.getValueAsDouble() / gearRatio;
        inputs.motorPosition = Rotation2d.fromRadians(Units.rotationsToRadians(motorPosition.getValueAsDouble()) / gearRatio);
        inputs.motorCurrentAmps = new double[] { motorCurrent.getValueAsDouble() };

        inputs.odometryMotorAccumulatedPosition = motorPositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryMotorPositions = motorPositionQueue.stream().map((Double value) -> Rotation2d.fromRotations(value / gearRatio)).toArray(Rotation2d[]::new);

        timestampQueue.clear();
        motorPositionQueue.clear();
    }

    @Override
    public void updateOutputs(SwerveMotorIOOutputs outputs) {
        outputs.motorAppliedVolts = motorAppliedVolts.getValueAsDouble();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(Units.rotationsToRadians(motorPosition.getValueAsDouble()) / gearRatio);
    }

    @Override
    public double getVelocity() {
        return motorVelocity.getValueAsDouble() / gearRatio;
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

        setVoltage(feedbackController.calculate(getAngle().getDegrees(), setpoint));
    }

    @Override
    public void setVelocity(double speedpoint) {
        if (!isDrive) throw new UnsupportedOperationException("Cannot set velocity on a turn motor");

        double velocityRPS = speedpoint / SwerveDrive.wheelRadius;
        setVoltage(drivFeedforward.calculate(velocityRPS) + feedbackController.calculate(getVelocity(), velocityRPS));
    }


    @Override
    public void setBrakeMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted = InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motor.getConfigurator().apply(config);
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        feedbackController.setPID(kP, kI, kD);
    }

    @Override
    public void configureFF(double kS, double kV, double kA) {
        drivFeedforward = new SimpleMotorFeedforward(0, kV, kA);
    }

    /**
     * Sets the Supply Current Limit of the motor
     * <br></br>
     * Default value is 40 Amps
     * @param limit In Amps
     */
    public void setCurrentLimit(double limit) {
        driveConfig.CurrentLimits.SupplyCurrentLimit = limit;
        motor.getConfigurator().apply(driveConfig);
    }

    /**
     * Sets the time to ramp from 0V to 12V output during closed-loop mode
     * <br></br>
     * Default value is 0.15 secs
     * @param period In seconds
     */
    public void setClosedLoopRampPeriod(double period) {
        motor.getConfigurator().apply(driveConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(period));
    }

    /**
     * Sets the update frequency of the Motor Position Status Signal to the specified frequency
     * <br></br>
     * Default value is 250 HZ
     * @param frequency In Hertz
     */
    public void setPositionUpdateFrequency(double frequency) {
        BaseStatusSignal.setUpdateFrequencyForAll(frequency, motorPosition);
    }

    /**
     * Sets the update frequency of the Motor Velocity, Motor Current, and Motor Applied Volts Status Signals
     * <br></br>
     * Default value is 50 HZ
     * @param frequency In hertz
     */
    public void setSignalUpdateFrequency(double frequency) {
        BaseStatusSignal.setUpdateFrequencyForAll(
            frequency,
            motorVelocity,
            motorCurrent,
            motorAppliedVolts
        );
    }
}
