package frc.robot.pioneersLib.bumSwerve.Encoder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class EncoderIOCANcoder implements EncoderIO {
    
    int ID;
    double CANcoderOffset;
    boolean inverted;

    CANcoder CANcoder;
    StatusSignal<Double> turnAbsolutePosition;
    CANcoderConfigurator configurator;
    MagnetSensorConfigs magnetSensorConfiguration;

    public EncoderIOCANcoder(int ID) {
        this.ID = ID;
        this.CANcoderOffset = 0;
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
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.absoluteEncoderOffset = CANcoderOffset;
        inputs.inverted = inverted;
    }

    @Override
    public void updateOutputs(EncoderIOOutputs outputs) {
        outputs.turnAbsolutePosition = turnAbsolutePosition.getValueAsDouble();
    }

    @Override
    public void setEncoderOffset(double offset) {
        this.CANcoderOffset = offset;
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


}
