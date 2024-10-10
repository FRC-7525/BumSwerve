package frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SwerveAbsoluteEncoderIOSim implements SwerveAbsoluteEncoderIO {

    private double absoluteEncoderOffset;
    private double encoderRotationDeg;
    private boolean inverted;

    private CANcoder dummyCANcoder;
    private CANcoderSimState CANcoderController;
    private CANcoderConfigurator configurator;
    private MagnetSensorConfigs magnetSensorConfiguration;
    private StatusSignal<Double> turnAbsolutePosition;
    


    SwerveAbsoluteEncoderIOSim(int ID) {
        this.absoluteEncoderOffset = 0;
        this.encoderRotationDeg = 0;
        this.inverted = false;

        this.dummyCANcoder = new CANcoder(ID);
        this.CANcoderController = dummyCANcoder.getSimState();
        this.turnAbsolutePosition = dummyCANcoder.getAbsolutePosition();
        this.configurator = dummyCANcoder.getConfigurator();
        this.magnetSensorConfiguration = new MagnetSensorConfigs();

        configurator.refresh(magnetSensorConfiguration);
        configurator.apply(magnetSensorConfiguration
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        );
    }
    @Override
    public void updateInputs(SwerveAbsoluteEncoderIOInputs inputs) {
        inputs.absoluteEncoderOffset = absoluteEncoderOffset;
        inputs.inverted = inverted;
        inputs.turnAbsolutePosition = encoderRotationDeg;
    }

    @Override
    public void setEncoderOffset(double offset) {
        this.absoluteEncoderOffset = offset;
        configurator.refresh(magnetSensorConfiguration);
        configurator.apply(magnetSensorConfiguration.withMagnetOffset(offset/360));
    }

    @Override
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
        configurator.refresh(magnetSensorConfiguration);
        configurator.apply(magnetSensorConfiguration
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withSensorDirection((inverted ? SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive))
        );
    }

    @Override
    public Rotation2d getTurnAbsolutePosition() {
        return new Rotation2d(Units.degreesToRadians(turnAbsolutePosition.getValueAsDouble()));
    }
    @Override
    public void setRotationDeg(double rotationDeg) {
        encoderRotationDeg = 0;
    }

    @Override
    public double getRotationDeg() {
        return encoderRotationDeg;
    }
}