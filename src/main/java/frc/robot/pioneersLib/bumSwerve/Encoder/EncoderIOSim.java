package frc.robot.pioneersLib.bumSwerve.Encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class EncoderIOSim implements EncoderIO {

    int ID;
    boolean inverted;
    double encoderOffset;

    EncoderSim encoder;

    public EncoderIOSim(int ID) {
        this.ID = ID;
        inverted = false;
        encoderOffset = 0;

        encoder = new EncoderSim(ID);
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.absoluteEncoderOffset = encoderOffset;
        inputs.inverted = inverted;
    }

    @Override
    public void updateOutputs(EncoderIOOutputs outputs) {
        outputs.turnAbsolutePosition = encoder.getRotationDeg();
    }

    @Override
    public void setEncoderOffset(double offset) {
        encoderOffset = offset;
        encoder.setEncoderOffset(offset);
    }

    @Override
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
        encoder.setInverted(inverted);
    }

    @Override
    public Rotation2d getTurnAbsolutePosition() {
        return new Rotation2d(Units.degreesToRadians(encoder.getRotationDeg()));
    }
    
}
