package frc.robot.pioneersLib.bumSwerve.SwerveMotorIOs;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.flatbuffers.Constants;

import frc.robot.pioneersLib.bumSwerve.OdometryThread;

public class SwerveMotorIOTalonFX implements SwerveMotorIO {
    private final TalonFX motor;

    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveCurrent;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> motorPositionQueue;

    public SwerveMotorIOTalonFX(int canID) {
        motor = new TalonFX(canID);

        drivePosition = motor.getPosition();
		driveVelocity = motor.getVelocity();
		driveAppliedVolts = motor.getMotorVoltage();
		driveCurrent = motor.getSupplyCurrent();

        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
		motorPositionQueue = OdometryThread.getInstance().registerSignal(motor, motor.getPosition());
    }

    @Override
    public void updateInputs(SwerveMotorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent
        );
    }
}
