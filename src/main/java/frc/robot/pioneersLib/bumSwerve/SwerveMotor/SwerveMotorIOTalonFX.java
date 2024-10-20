package frc.robot.pioneersLib.bumSwerve.SwerveMotor;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.pioneersLib.bumSwerve.OdometryThread;

public class SwerveMotorIOTalonFX implements SwerveMotorIO {
    private final TalonFX motor;

    private final StatusSignal<Double> motorPosition;
    private final StatusSignal<Double> motorVelocity;
    private final StatusSignal<Double> motorAppliedVolts;
    private final StatusSignal<Double> motorCurrent;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> motorPositionQueue;

    private double gearRatio;
    private boolean isDrive;

    private TalonFXConfiguration driveConfig;
    private TalonFXConfigurator configurator;
    private Slot0Configs configs;

    private double positionError;

    //Default config values
    private static final double CURRENT_LIMIT = 40.0;
    private static final double POSITION_UPDATE_FREQUENCY = 250.0;
    private static final double SIGNAL_UPDATE_FREQUENCY = 50.0;
    private static final double VOLTAGE_CLOSED_LOOP_RAMP_PERIOD = 0.15;

    public SwerveMotorIOTalonFX(int canID, double gearRatio) {
        this.gearRatio = gearRatio;

        configs = new Slot0Configs();

        motor = new TalonFX(canID);

        configurator = motor.getConfigurator();

        driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        configurator.apply(driveConfig);
        //TODO: Apparently this might not work (idk what it's even supposed to do)
        configurator.apply(driveConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(VOLTAGE_CLOSED_LOOP_RAMP_PERIOD)); 

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

        configs.kP = 0.0;
        configs.kI = 0.0;
        configs.kD = 0.0;
        configurator.apply(configs);

        FeedbackConfigs feedback = new FeedbackConfigs();
        feedback.SensorToMechanismRatio = gearRatio;
        configurator.apply(feedback);

        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
		motorPositionQueue = OdometryThread.getInstance().registerSignal(motor, motor.getPosition());
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

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryMotorPositions = motorPositionQueue.stream().map((Double value) -> Rotation2d.fromRotations(value / gearRatio)).toArray(Rotation2d[]::new);

        if (isDrive) inputs.odometryDriveAccumulatedPosition = motorPositionQueue.stream().mapToDouble((Double value) -> value / gearRatio).toArray();

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
    public void setEncoderPosition(double positionDeg) {
        motor.setPosition(positionDeg/360);
    }

    @Override
    public double getPositionError() {
        return positionError;
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setPosition(double setpoint) {
        if (isDrive) throw new UnsupportedOperationException("Cannot set position on a drive motor");

        PositionVoltage command = new PositionVoltage(setpoint/360).withSlot(0);
        motor.setControl(command);

        positionError = Math.abs(setpoint/360 - motorPosition.getValueAsDouble());
    }

    @Override
    public void setVelocity(double speedpoint) {
        if (!isDrive) throw new UnsupportedOperationException("Cannot set velocity on a turn motor");

        VelocityVoltage command = new VelocityVoltage(speedpoint).withSlot(0);
        motor.setControl(command);
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
        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;

        configurator.apply(configs);
    }

    @Override
    public void configureFF(double kS, double kV, double kA) {
        configs.kS = kS;
        configs.kV = kV;
        configs.kA = kA;

        configurator.apply(configs);
    }

    @Override
    public void setIsDrive(boolean isDrive) {
        this.isDrive = isDrive;
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

    @Override
    public void runVolt(double volts) {
        motor.setVoltage(volts);
    }
}
