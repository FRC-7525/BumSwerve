package frc.robot.pioneersLib.bumSwerve.SwerveMotorIOs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import java.util.OptionalDouble;
import java.util.Queue;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.pioneersLib.bumSwerve.OdometryThread;

public class SwerveMotorIONeoSim implements SwerveMotorIO {
    private REVPhysicsSim revSim;
    private CANSparkMax dummySpark;
    private SparkPIDController controller;
    private RelativeEncoder encoder;
    private DCMotorSim motorSim;

    private double gearRatio;
    private boolean isDrive;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> motorPositionQueue;

    public SwerveMotorIONeoSim(int id, boolean isDrive, double gearRatio) {
        revSim = REVPhysicsSim.getInstance();
        dummySpark = new CANSparkMax(id, MotorType.kBrushless);
        controller = dummySpark.getPIDController();

        motorSim = new DCMotorSim(DCMotor.getNEO(1), gearRatio, isDrive ? 0.000520786 : 0.00062093);
        motorSim.update(0.02);
        encoder = dummySpark.getEncoder();

        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        motorPositionQueue = OdometryThread.getInstance().registerSignal(() -> {
            double value = encoder.getPosition();
            if (dummySpark.getLastError() == REVLibError.kOk) {
                return OptionalDouble.of(value);
            } else {
                return OptionalDouble.of(0);
            }
        });

        revSim.addSparkMax(dummySpark, DCMotor.getNEO(1));
    }

    public void updateInputs(SwerveMotorIOInputs inputs) {
        revSim.run();
        inputs.motorPosition = Rotation2d.fromRotations(encoder.getPosition());
        inputs.motorCurrentAmps = new double[] {dummySpark.getOutputCurrent()};
        inputs.motorVelocityRPS = encoder.getVelocity();

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryMotorPositions = motorPositionQueue.stream().map((Double value) -> Rotation2d.fromRotations(value)).toArray(Rotation2d[]::new);

    }

    public void updateOutputs(SwerveMotorIOOutputs outputs) {
        outputs.motorAppliedVolts = dummySpark.getAppliedOutput();
    }

    public void setVoltage(double volts) {
        dummySpark.setVoltage(volts);
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        controller.setP(kD, 0);
        controller.setI(kI, 0);
        controller.setD(kD, 0);
    }
    
    public void configureFF(double kV) {
        // TODO: What ff value is this???? I'm assuming it's kV or the sum of kS and kV??
        controller.setFF(kV, 0);
    }

    @Override
    public void setVelocity(double velocityRPS) {
        if (!isDrive) throw new UnsupportedOperationException("Cannot set velocity on a turn motor");
        controller.setReference(velocityRPS, ControlType.kVelocity);
    }

    @Override
    public void setPosition(double positionDeg) {
        if (isDrive) throw new UnsupportedOperationException("Cannot set position on a drive motor");
        controller.setReference(positionDeg / 360, ControlType.kPosition, 0);
    }
}