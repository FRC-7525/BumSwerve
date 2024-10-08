package frc.robot.pioneersLib.bumSwerve.Gyro;

import java.util.OptionalDouble;
import java.util.Queue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.simulation.ADIS16448_IMUSim;
import frc.robot.pioneersLib.bumSwerve.OdometryThread;

/*
 * IO implementation for the ADIS16448 IMU, allows for gyro simulation
 * using a wpilib sim so we don't have to rely on module positions for
 * robot rotation. Everything is in deg
 */

public class SwerveGyroIOSim implements SwerveGyroIO {
    // Most comparable to NavX out of what wpilib sim offers
    private final ADIS16448_IMU gyro;
    private final ADIS16448_IMUSim gyroController;

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    /**
     * Creates a new simulated gyro object using the ADIS16448 IMU Sim
     */
    public SwerveGyroIOSim() {
        gyro = new ADIS16448_IMU();
        gyroController = new ADIS16448_IMUSim(gyro);

        yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = OdometryThread.getInstance()
                .registerSignal(() -> {
                    if (gyro.isConnected()) {
                        return OptionalDouble.of(gyro.getAngle());
                    } else {
                        return OptionalDouble.empty();
                    }
                });
    }

    /**
     * Zeroes simulation gyro's rotation on all axes directly
     */
    @Override
    public void zero() {
        gyroController.setGyroAngleX(0);
        gyroController.setGyroAngleY(0);
        gyroController.setGyroAngleZ(0);
    }

    @Override
    public void updateInputs(SwerveGyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(gyro.getAngle());
        inputs.yawVelocityRotPerSec = gyro.getRate()/360;

        if (yawTimestampQueue != null && yawPositionQueue != null) {
            inputs.odometryYawTimestamps = yawTimestampQueue
                    .stream()
                    .mapToDouble((Double value) -> value)
                    .toArray();
            inputs.odometryYawPositions = yawPositionQueue
                    .stream()
                    .map((Double value) -> Rotation2d.fromDegrees(value))
                    .toArray(Rotation2d[]::new);

            yawTimestampQueue.clear();
            yawPositionQueue.clear();
        }
    }
}
