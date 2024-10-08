package frc.robot.pioneersLib.bumSwerve.Gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.pioneersLib.bumSwerve.OdometryThread;

import java.util.OptionalDouble;
import java.util.Queue;

/** IO implementation for NavX-MXP */
public class SwerveGyroIONavX implements SwerveGyroIO {

	private final AHRS navx;
	private final Queue<Double> yawPositionQueue;
	private final Queue<Double> yawTimestampQueue;

	private Rotation3d offset;
	/**
	 * Creates a new NavX objet that takes high frequency odometry measurments
	 */
	public SwerveGyroIONavX() {
		// TODO: Change it so u don't have to plug into specific usb port
		navx = new AHRS(SerialPort.Port.kUSB1);
		navx.reset();
		offset = new Rotation3d();

		yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
		yawPositionQueue = OdometryThread.getInstance()
			.registerSignal(() -> {
				if (navx.isConnected()) {
					return OptionalDouble.of(Units.degreesToRadians(navx.getRotation3d().minus(offset).getAngle()));
				} else {
					return OptionalDouble.empty();
				}
			});
	}

	/**
	 * Zeroes the gyro yaw position with an offset
	 */
	@Override
	public void zero() {
		this.offset = navx.getRotation3d();
	}

	@Override
	public void updateInputs(SwerveGyroIOInputs inputs) {
		inputs.connected = navx.isConnected();
		inputs.yawPosition = Rotation2d.fromRadians(navx.getRotation3d().minus(offset).getAngle());
		inputs.yawVelocityRPS = navx.getRate()/360;

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
