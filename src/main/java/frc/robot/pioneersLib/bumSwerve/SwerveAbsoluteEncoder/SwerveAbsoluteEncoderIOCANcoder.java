package frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SwerveAbsoluteEncoderIOCANcoder implements SwerveAbsoluteEncoderIO {
    
    private double absoluteEncoderOffset;
    private boolean inverted;

    private CANcoder CANcoder;
    private StatusSignal<Double> turnAbsolutePosition;
    private CANcoderConfigurator configurator;
    private MagnetSensorConfigs magnetSensorConfiguration;

    public SwerveAbsoluteEncoderIOCANcoder(int ID) {
        this.absoluteEncoderOffset = 0;
        this.inverted = false;
        
        this.CANcoder = new CANcoder(ID);
        this.turnAbsolutePosition = CANcoder.getAbsolutePosition();
        this.configurator = CANcoder.getConfigurator();
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
        inputs.turnAbsolutePosition = turnAbsolutePosition.getValueAsDouble();
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

    @Override //does nothing because it is only used in sim?
    public void setRotationDeg(double rotationDeg) {
        return;
    }

    @Override
    public double getRotationDeg() {
        return turnAbsolutePosition.getValueAsDouble();
    }
}
