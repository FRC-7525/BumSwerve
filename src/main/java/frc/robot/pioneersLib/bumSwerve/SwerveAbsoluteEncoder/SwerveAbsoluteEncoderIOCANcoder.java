package frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CAN;

public class SwerveAbsoluteEncoderIOCANcoder implements SwerveAbsoluteEncoderIO {
    
    private double absoluteEncoderOffset;
    private boolean inverted;

    // private StatusSignal<Double> turnAbsolutePosition;
    private CANcoderConfigurator configurator;
    private CANcoder CaNcoder;
    private MagnetSensorConfigs magnetSensorConfiguration;

    public SwerveAbsoluteEncoderIOCANcoder(int ID, double encoderOffset) {
        this.absoluteEncoderOffset = encoderOffset;
        this.inverted = false;
        
        CaNcoder = new CANcoder(ID);
            // this.turnAbsolutePosition = CaNcoder.getAbsolutePosition();
        // this.configurator = CaNcoder.getConfigurator();
        
        // this.magnetSensorConfiguration = new MagnetSensorConfigs();

        // configurator.apply(new CANcoderConfiguration());

        // configurator.refresh(magnetSensorConfiguration);
        // configurator.apply(magnetSensorConfiguration
        //     .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
        //     .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        // );
        // configurator.apply(magnetSensorConfiguration.withMagnetOffset(encoderOffset/360));
        
    }

    @Override
    public void updateInputs(SwerveAbsoluteEncoderIOInputs inputs) {
        // BaseStatusSignal.refreshAll(turnAbsolutePosition);

        inputs.absoluteEncoderOffset = absoluteEncoderOffset;
        inputs.inverted = inverted;
        inputs.turnAbsolutePosition = (CaNcoder.getAbsolutePosition().getValueAsDouble() * 360) + absoluteEncoderOffset;
    }

    @Override
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
        // configurator.refresh(magnetSensorConfiguration);
        // configurator.apply(magnetSensorConfiguration
        //     .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
        //     .withSensorDirection((inverted ? SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive))
        // );
    }

    @Override
    public Rotation2d getTurnAbsolutePosition() {
        return Rotation2d.fromRotations(CaNcoder.getAbsolutePosition().getValueAsDouble()).plus(Rotation2d.fromDegrees(absoluteEncoderOffset));
        // return new Rotation2d();
    }

    @Override //does nothing because it is only used in sim?
    public void setRotationDeg(double rotationDeg) {
        return;
    }

    @Override
    public double getRotationDeg() {
        return getTurnAbsolutePosition().getDegrees();
    }

    @Override
    public boolean isSim() {
        return false;
    }
}
