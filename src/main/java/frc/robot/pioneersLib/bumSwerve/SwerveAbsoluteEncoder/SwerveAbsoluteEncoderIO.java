package frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveAbsoluteEncoderIO {
    @AutoLog
    public class SwerveAbsoluteEncoderIOInputs {
        public double encoderPosition = 0.0;
    }

    public default double getRotationDeg() {
        return 0.0;
    }
}
