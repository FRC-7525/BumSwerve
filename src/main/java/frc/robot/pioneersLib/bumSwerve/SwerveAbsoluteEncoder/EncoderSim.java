package frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder;


public class EncoderSim {
    int ID;
    double encoderRotationDeg;
    double encoderOffset;
    boolean inverted;

    EncoderSim(int ID) {
        this.ID = ID;
        encoderRotationDeg = 0;
        encoderOffset = 0;
        inverted = false;
    }

    public void setRotationDeg(double deg) {
        this.encoderRotationDeg = deg;
    }

    public void setEncoderOffset (double deg) {
        this.encoderOffset = deg;
    }

    public double getRotationDeg() {
        return this.encoderRotationDeg;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

}
