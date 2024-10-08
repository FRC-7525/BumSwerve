package frc.robot.pioneersLib.bumSwerve.SwerveMotorIOs;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveMotorIOTalonFXSim implements SwerveMotorIO {

    private TalonFX dummyTalon;
    private TalonFXSimState talonController;
    private DCMotorSim motorSim;
    private double gearing;
    private TalonFXConfigurator configurator;

    private StatusSignal motorVoltage;
    
    public SwerveMotorIOTalonFXSim(int placeholderCanID, double gearing) {
        dummyTalon = new TalonFX(placeholderCanID);
        talonController = dummyTalon.getSimState();    
        configurator = dummyTalon.getConfigurator();
        motorSim.update(0.02);

        this.gearing = gearing;

        //TODO: Find out wth jkg meters squared means and ask mech to calculate it or use the other constructor
        motorSim = new DCMotorSim(DCMotor.getKrakenX60(1), gearing, 1);

        BaseStatusSignal.setUpdateFrequencyForAll(50, motorVoltage);
    }  

    public void periodic() {
        BaseStatusSignal.refreshAll(motorVoltage);
    }

    @Override
    public void setVoltage(double volts) {
        dummyTalon.setVoltage(volts);
    }

    public void setVelocity(double velocityRPS) {
        VelocityVoltage command = new VelocityVoltage(velocityRPS).withSlot(0);
        dummyTalon.setControl(command);
        motorSim.setInputVoltage(talonController.getMotorVoltage());
    }
}
