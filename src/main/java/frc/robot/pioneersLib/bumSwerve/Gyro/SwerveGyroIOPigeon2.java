package frc.robot.pioneersLib.bumSwerve.Gyro;

import java.util.OptionalDouble;
import java.util.Queue;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.pioneersLib.bumSwerve.OdometryThread;

public class SwerveGyroIOPigeon2 implements SwerveGyroIO{

    private final Pigeon2 gyro;
	private final Queue<Double> yawPositionQueue;
	private final Queue<Double> yawTimestampQueue;
    private Rotation3d offset;

    public SwerveGyroIOPigeon2(int canId) {
        gyro = new Pigeon2(canId);

        gyro.reset();
        offset = new Rotation3d();

        yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
		yawPositionQueue = OdometryThread.getInstance()
            .registerSignal(() -> {
                if(gyro.getUpTime().getValue() != 0) {
                    return OptionalDouble.of(gyro.getRotation3d().minus(offset).getAngle());
                } else {
                    return OptionalDouble.empty();
                }
            });
    }

    @Override
    public void zero() {
        this.offset = gyro.getRotation3d();
    }

    @Override
	public void updateInputs(SwerveGyroIOInputs inputs) {
		inputs.connected = (gyro.getUpTime().getValue() != 0);
		inputs.yawPosition = Rotation2d.fromRadians(gyro.getRotation3d().minus(offset).getAngle());
		inputs.yawVelocityRPS = gyro.getRate();

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
