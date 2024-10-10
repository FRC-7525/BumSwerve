package frc.robot.pioneersLib.bumSwerve.Encoder;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface EncoderIO {

    @AutoLog
    public static class EncoderIOInputs {
        public double absoluteEncoderOffset;
        public boolean inverted;
    }

    public static class EncoderIOOutputs {
        public double turnAbsolutePosition;
    }

    public void updateInputs(EncoderIOInputs inputs);

    public void updateOutputs(EncoderIOOutputs outputs);

    public void setEncoderOffset(double offset);

    public void setInverted(boolean inverted);

    public Rotation2d getTurnAbsolutePosition();

}
