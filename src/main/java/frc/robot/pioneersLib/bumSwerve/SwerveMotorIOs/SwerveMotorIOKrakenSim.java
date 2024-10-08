package frc.robot.pioneersLib.bumSwerve.SwerveMotorIOs;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveMotorIOKrakenSim implements SwerveMotorIO {

    private TalonFX dummyTalon;
    private TalonFXSimState talonController;
    private TalonFXConfigurator configurator;

    private DCMotorSim motorSim;
    private Slot0Configs configs;

    private double gearing;
    private boolean isDrive;
    
    public SwerveMotorIOKrakenSim(int placeholderCanID, boolean isDrive, double gearRatio) {
        dummyTalon = new TalonFX(placeholderCanID);
        talonController = dummyTalon.getSimState();    
        configurator = dummyTalon.getConfigurator();

        // Note: These MOI values are for L3+ SDS Swerve Modules w kraken drive & neo turn
        motorSim = new DCMotorSim(DCMotor.getKrakenX60(1), gearing, isDrive ? 0.000520786 : 0.00062093);
        motorSim.update(0.02);

        configs = new Slot0Configs();

        // TODO: See if dummy PID is uneeded
        configs.kP = 0.0;
        configs.kI = 0.0;
        configs.kD = 0.0;
        configurator.apply(configs);

        // Take into account gear ratio
        FeedbackConfigs feedback = new FeedbackConfigs();
        // TODO: 1 or gear ratio? do u need to use gear ratio if u already do in the sim configs?
        feedback.SensorToMechanismRatio = gearRatio;
        configurator.apply(feedback);

        this.gearing = gearRatio;
        this.isDrive = isDrive;
    }  

    @Override
    public void setVoltage(double volts) {
        dummyTalon.setVoltage(volts);
    }
    
    @Override
    public void configurePID(double kP, double kI, double kD) {
        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;

        configurator.apply(configs);
    }

    public void configureFF(double kS, double kV) {
        configs.kS = kS;
        configs.kV = kV;

        configurator.apply(configs);
    }

    @Override
    public void setVelocity(double velocityRPS) {
        if (!isDrive) throw new Error("Cannot use setVelocity with a turn motor");

        VelocityVoltage command = new VelocityVoltage(velocityRPS).withSlot(0);
        dummyTalon.setControl(command);
        motorSim.setInputVoltage(talonController.getMotorVoltage());
        talonController.setRawRotorPosition(motorSim.getAngularPositionRotations());
        talonController.setRotorVelocity(motorSim.getAngularVelocityRPM()/60);
    }

    @Override
    public void setPosition(double positionDeg) {
        if (isDrive) throw new Error("Cannot use setPosition with a drive motor");

        PositionVoltage command = new PositionVoltage(positionDeg/360).withSlot(0);
        dummyTalon.setControl(command);
        motorSim.setInputVoltage(talonController.getMotorVoltage());
        talonController.setRawRotorPosition(motorSim.getAngularPositionRotations());
        talonController.setRotorVelocity(motorSim.getAngularVelocityRPM()/60);
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
    
    

}
