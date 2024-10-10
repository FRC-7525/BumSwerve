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
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.pioneersLib.bumSwerve.OdometryThread;
import frc.robot.pioneersLib.bumSwerve.SwerveModule;

public class SwerveMotorIONeoSim implements SwerveMotorIO {
    private REVPhysicsSim revSim;
    private CANSparkMax dummySpark;
    private SparkPIDController controller;
    private RelativeEncoder encoder;

    private boolean isDrive;

    private double positionError;

    private PIDController turnController;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> motorPositionQueue;

    // Default values
    private final boolean MOTOR_INVERTED = false;
    private final int SPARK_TIMEOUT_MS = 250;
    private final int MOTOR_CURRENT_LIMIT = 30;
    private final double MAX_VOLTS = 12.0;
    private final int SPARK_MEASUREMENT_PERIOD_MS = 10;
    private final double SPARK_FRAME_PERIOD = 1000.0 / SwerveModule.ODOMETRY_FREQUENCY;
    private final double RAMP_RATE = 0.16;

    public SwerveMotorIONeoSim(int placeholderCANId, double gearRatio) {
        revSim = REVPhysicsSim.getInstance();
        dummySpark = new CANSparkMax(placeholderCANId, MotorType.kBrushless);
        revSim.addSparkMax(dummySpark, DCMotor.getNEO(1));

        controller = dummySpark.getPIDController();
        turnController = new PIDController(0, 0, 0);

        encoder = dummySpark.getEncoder();
        encoder.setPosition(0);

        dummySpark.restoreFactoryDefaults();
        encoder.setMeasurementPeriod(
			SPARK_MEASUREMENT_PERIOD_MS
		);
        dummySpark.setPeriodicFramePeriod(
                    PeriodicFrame.kStatus2,
                    (int) (SPARK_FRAME_PERIOD)
                );	
        // TODO: Why does AKIT do this??	
        dummySpark.setCANTimeout(SPARK_TIMEOUT_MS);
        dummySpark.setCANTimeout(0);

		dummySpark.setInverted(MOTOR_INVERTED);
		dummySpark.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
		dummySpark.enableVoltageCompensation(MAX_VOLTS);
		dummySpark.setClosedLoopRampRate(RAMP_RATE);

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

        timestampQueue.clear();
		motorPositionQueue.clear();
    }

    public void updateOutputs(SwerveMotorIOOutputs outputs) {
        outputs.motorAppliedVolts = dummySpark.getAppliedOutput();
    }

    public void setVoltage(double volts) {
        dummySpark.setVoltage(volts);
    }   

    @Override
    public void setEncoderPosition(double positionDeg) {
        encoder.setPosition(positionDeg / 360);
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
        if (isDrive) {
            // slot 1 pos, 2 vel, 3 misc, 0 default??
            controller.setP(kP, 0);
            controller.setI(kI, 0);
            controller.setD(kD, 0);

            dummySpark.burnFlash();
        } else if (!isDrive) {
            turnController.setP(kP);
            turnController.setI(kI);
            turnController.setD(kD);
        }
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
        revSim.run();

        if (isDrive) throw new UnsupportedOperationException("Cannot set position on a drive motor");

        setVoltage(turnController.calculate(getAngle().getRotations(), positionDeg / 360));
        positionError = Math.abs(positionDeg/360 - encoder.getPosition());
    }

    public void setInverted(boolean inverted) {
        dummySpark.setInverted(inverted);
    }

    @Override
    public void setIsDrive(boolean isDrive) {
        this.isDrive = isDrive;
    }
}
