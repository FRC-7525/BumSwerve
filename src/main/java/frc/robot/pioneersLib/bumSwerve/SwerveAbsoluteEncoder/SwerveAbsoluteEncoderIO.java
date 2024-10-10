package frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveAbsoluteEncoderIO{

    @AutoLog
    public static class SwerveAbsoluteEncoderIOInputs {
        public double absoluteEncoderOffset;
        public boolean inverted;
        public double turnAbsolutePosition;
    }

    public void updateInputs(SwerveAbsoluteEncoderIOInputs inputs);

    /*Sets encoder offset*/
    public void setEncoderOffset(double offset);

    /*Inverts the encoder. True is CW, False is CCW */
    public void setInverted(boolean inverted);

    /*Sets Rotation degree only used in sim (i think) */
    public void setRotationDeg(double rotationDeg);

    /*Gets rotation degree as a Rotation2d */
    public Rotation2d getTurnAbsolutePosition();
    
    /*Returns position status signal */
    public StatusSignal<Double> getRotationDeg();

    public boolean isSim();


}
