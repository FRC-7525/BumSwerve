package frc.robot.pioneersLib.bumSwerve.SwerveMotor;

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
import frc.robot.pioneersLib.bumSwerve.OdometryThread;

public class SwerveMotorIONeoSim implements SwerveMotorIO {
    private REVPhysicsSim revSim;
    private CANSparkMax dummySpark;
    private SparkPIDController controller;
    private RelativeEncoder encoder;

    private boolean isDrive;

    private double positionError;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> motorPositionQueue;

    // TODO: Do sum about rev sim being ass

    public SwerveMotorIONeoSim(int placeholderCANId, boolean isDrive, double gearRatio) {
        revSim = REVPhysicsSim.getInstance();
        dummySpark = new CANSparkMax(placeholderCANId, MotorType.kBrushless);
        revSim.addSparkMax(dummySpark, DCMotor.getNEO(1));

        controller = dummySpark.getPIDController();

        encoder = dummySpark.getEncoder();
        encoder.setPosition(0);

        dummySpark.restoreFactoryDefaults();
		dummySpark.setCANTimeout(0);

		dummySpark.setInverted(false);
		dummySpark.setSmartCurrentLimit(40);
		dummySpark.enableVoltageCompensation(12);
		dummySpark.setClosedLoopRampRate(0.15);

        revSim.run();

        dummySpark.burnFlash();

        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        motorPositionQueue = OdometryThread.getInstance().registerSignal(() -> {
            double value = encoder.getPosition();
            if (dummySpark.getLastError() == REVLibError.kOk) {
                return OptionalDouble.of(value);
            } else {
                return OptionalDouble.of(0);
            }
        });
    }

    public void updateInputs(SwerveMotorIOInputs inputs) {
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
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(encoder.getPosition());
    }

    @Override
    public double getPositionError() {
        return positionError;
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {

        // slot 1 pos, 2 vel, 3 misc, 0 default??
        controller.setP(kP, 0);
        controller.setI(kI, 0);
        controller.setD(kD, 0);

        dummySpark.burnFlash();
    }
    
    @Override
    public void configureFF(double kS, double kV, double kA) {
        // TODO: What ff value is this???? I'm assuming it's kV or the sum of kS and kV??, I hate rev sooooooooo much
        controller.setFF(kV, 0);
        dummySpark.burnFlash();
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
        // System.out.println(positionDeg);
        // System.out.println(positionError);
        positionError = Math.abs(positionDeg/360 - encoder.getPosition());
    }

    public void setInverted(boolean inverted) {
        dummySpark.setInverted(inverted);
    }
}
