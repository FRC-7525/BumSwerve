package frc.robot.pioneersLib.bumSwerve.SwerveMotor;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.pioneersLib.bumSwerve.OdometryThread;

public class SwerveMotorIOTalonFXSim implements SwerveMotorIO {

    private TalonFX dummyTalon;
    private TalonFXSimState talonController;
    private TalonFXConfigurator configurator;

    private DCMotorSim motorSim;
    private Slot0Configs configs;

    private PIDController pidController;

    private StatusSignal<Double> motorPosition;
    private StatusSignal<Double> motorVelocityRPS;
    private StatusSignal<Double> motorCurrent;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> motorPositionQueue;

    private double gearing;
    private double positionError;
    private boolean isDrive;

    //Default configs
    private final double POSITION_UPDATE_FREQUENCY = 250.0;
    private final double SIGNAL_UPDATE_FREQUENCY = 50.0;
    private final double DT_TIME = 0.02;
    
    public SwerveMotorIOTalonFXSim(int placeholderCanID, double gearRatio, double motorMOI, DCMotor motorType) {
        dummyTalon = new TalonFX(placeholderCanID);
        talonController = dummyTalon.getSimState();    
        configurator = dummyTalon.getConfigurator();

        this.gearing = gearRatio;

        motorSim = new DCMotorSim(motorType, gearing, motorMOI);

        configs = new Slot0Configs();

        configs.kP = 0.0;
        configs.kI = 0.0;
        configs.kD = 0.0;
        configurator.apply(configs);
        
        FeedbackConfigs feedback = new FeedbackConfigs();
        feedback.SensorToMechanismRatio = 1/gearRatio;
        configurator.apply(feedback);
        pidController = new PIDController(0, 0, 0);

        motorPosition = dummyTalon.getPosition();
        motorVelocityRPS = dummyTalon.getVelocity();
        motorCurrent = dummyTalon.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(POSITION_UPDATE_FREQUENCY, motorPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
			SIGNAL_UPDATE_FREQUENCY,
			motorVelocityRPS,
			motorCurrent
		);

        positionError = 0.0;

        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
		motorPositionQueue = OdometryThread.getInstance().registerSignal(dummyTalon, motorPosition);
    }  
    
    public void updateInputs(SwerveMotorIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorPosition, motorVelocityRPS, motorCurrent);

        inputs.motorPosition = Rotation2d.fromRotations(motorPosition.getValueAsDouble());
        inputs.motorVelocityRPS = motorVelocityRPS.getValueAsDouble();
        inputs.motorCurrentAmps = new double[] {motorCurrent.getValueAsDouble()};

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryMotorPositions = motorPositionQueue.stream().map((Double value) -> Rotation2d.fromRotations(value)).toArray(Rotation2d[]::new);

        if (isDrive) inputs.odometryDriveAccumulatedPosition = motorPositionQueue.stream().mapToDouble((Double value) -> value).toArray();

        timestampQueue.clear();
		motorPositionQueue.clear();
    }

    public void updateOutputs(SwerveMotorIOOutputs outputs) {
        outputs.motorAppliedVolts = talonController.getMotorVoltage();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(dummyTalon.getPosition().getValueAsDouble());
    }

    @Override
    public double getPositionError() {
        return positionError;
    }

    @Override
    public void setVoltage(double volts) {
        dummyTalon.setVoltage(volts);
    }
    
    @Override
    public void setEncoderPosition(double positionDeg) {
        dummyTalon.setPosition(positionDeg/360);
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;
        pidController.setPID(kP, kI, kD);

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
    public void setVelocity(double velocityRPS) {
        motorSim.update(DT_TIME);
        if (!isDrive) throw new Error("Cannot use setVelocity with a turn motor");

        VelocityVoltage command = new VelocityVoltage(velocityRPS).withSlot(0);
        dummyTalon.setControl(command);
        motorSim.setInputVoltage(talonController.getMotorVoltage());
        talonController.setRawRotorPosition(motorSim.getAngularPositionRotations()/gearing);
        talonController.setRotorVelocity(motorSim.getAngularVelocityRPM()/(60*gearing));
        // System.out.println((dummyTalon.getVelocity().getValueAsDouble() - velocityRPS) * Math.PI * 2 * Units.inchesToMeters(2));
    }

    @Override
    public void setPosition(double positionDeg) {
        if (isDrive) throw new Error("Cannot use setPosition with a drive motor");
        motorSim.update(DT_TIME);

        PositionVoltage command = new PositionVoltage(positionDeg/360).withSlot(0);
        dummyTalon.setControl(command);
        motorSim.setInputVoltage(talonController.getMotorVoltage());
        talonController.setRawRotorPosition(motorSim.getAngularPositionRotations()/gearing);
        talonController.setRotorVelocity(motorSim.getAngularVelocityRPM()/(60 * gearing));

        positionError = Math.abs(positionDeg/360 - (dummyTalon.getPosition().getValueAsDouble()));
    }

    /**
     * Enable or disable brake mode on the motor
     * <br></br>
     * Default is coast
     */
    @Override
    public void setBrakeMode(boolean enableBreak) {
        dummyTalon.setNeutralMode(enableBreak ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setIsDrive(boolean isDrive) {
        this.isDrive = isDrive;
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
     * Sets the update frequency of the Motor Velocity, and Motor Current Status Signals
     * <br></br>
     * Default value is 50 HZ
     * @param frequency In hertz
     */
    public void setSignalUpdateFrequency(double frequency) {
        BaseStatusSignal.setUpdateFrequencyForAll(
            frequency,
            motorVelocityRPS,
            motorCurrent
        );
    }
}
