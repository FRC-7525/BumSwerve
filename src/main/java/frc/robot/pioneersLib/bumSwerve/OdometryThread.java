package frc.robot.pioneersLib.bumSwerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class OdometryThread extends Thread {

	private final Lock signalsLock = new ReentrantLock();
	private List<BaseStatusSignal> phoenixSignals = new ArrayList<>();
	private List<Supplier<OptionalDouble>> polledSignals = new ArrayList<>();
	private List<Queue<Double>> queues = new ArrayList<>();
	private List<Queue<Double>> timestampQueues = new ArrayList<>();

	// Ensures samples added to correct queue (0 for phoenix, 1 for polled)
	private List<Integer> signalTypes = new ArrayList<>();

	private static final int PHOENIX_SIGNAL = 0;
	private static final int POLLED_SIGNAL = 1;

	private static final int MAX_QUEUE_SIZE = 20;

	private static final double PHOENIX_SIGNAL_TIMEOUT = 2.0 / 250.0;
	private static final double PHOENIX_EMPTY_WAIT_TIME = 1000.0 / 250.0;

	private static OdometryThread instance = null;

	public static OdometryThread getInstance() {
		if (instance == null) {
			instance = new OdometryThread();
		}
		return instance;
	}

	private OdometryThread() {
		setName("HybridOdometryThread");
		setDaemon(true);
	}

	@Override
	public void start() {
		if (timestampQueues.size() > 0) {
			super.start();
		}
	}

	public Queue<Double> registerSignal(ParentDevice device, StatusSignal<Double> signal) {
		Queue<Double> queue = new ArrayBlockingQueue<>(
			MAX_QUEUE_SIZE
		);
		signalsLock.lock();
		SwerveDrive.odometryLock.lock();
		try {
			phoenixSignals.add(signal);
			queues.add(queue);
			signalTypes.add(PHOENIX_SIGNAL);
		} finally {
			signalsLock.unlock();
			SwerveDrive.odometryLock.unlock();
		}
		return queue;
	}
	
	public Queue<Double> registerSignal(Supplier<OptionalDouble> signal) {
		Queue<Double> queue = new ArrayBlockingQueue<>(
			MAX_QUEUE_SIZE
		);
		SwerveDrive.odometryLock.lock();
		try {
			polledSignals.add(signal);
			queues.add(queue);
			signalTypes.add(POLLED_SIGNAL);
		} finally {
			SwerveDrive.odometryLock.unlock();
		}
		return queue;
	}

	public Queue<Double> makeTimestampQueue() {
		Queue<Double> queue = new ArrayBlockingQueue<>(
			MAX_QUEUE_SIZE
		);
		SwerveDrive.odometryLock.lock();
		try {
			timestampQueues.add(queue);
		} finally {
			SwerveDrive.odometryLock.unlock();
		}
		return queue;
	}

	@Override
	public void run() {
		while (true) {
			signalsLock.lock();
			try {
				if (!phoenixSignals.isEmpty()) {
					BaseStatusSignal.waitForAll(
						PHOENIX_SIGNAL_TIMEOUT,
						phoenixSignals.toArray(new BaseStatusSignal[0])
					);
				} else {
					Thread.sleep((long) (PHOENIX_EMPTY_WAIT_TIME));
				}
			} catch (InterruptedException e) {
				e.printStackTrace();
			} finally {
				signalsLock.unlock();
			}

			SwerveDrive.odometryLock.lock();
			try {
				double timestamp = Logger.getRealTimestamp() / 1e6;
				double totalLatency = 0.0;
				for (BaseStatusSignal signal : phoenixSignals) {
					totalLatency += signal.getTimestamp().getLatency();
				}
				if (!phoenixSignals.isEmpty()) {
					timestamp -= totalLatency / phoenixSignals.size();
				}

				// If 1 thing breaks everything breaks, probably a bad idea idrc
				boolean allValid = true;
				int phoenixIndex = 0;
				int polledIndex = 0;

				for (int i = 0; i < signalTypes.size(); i++) {
					double value;
					if (signalTypes.get(i) == PHOENIX_SIGNAL) {
						value = phoenixSignals.get(phoenixIndex++).getValueAsDouble();
					} else {
						OptionalDouble optionalValue = polledSignals.get(polledIndex++).get();
						if (optionalValue.isPresent()) {
							value = optionalValue.getAsDouble();
						} else {
							allValid = false;
							break;
						}
					}

					if (allValid) {
						queues.get(i).offer(value);
					}
				}

				// All thingies sampled together
				if (allValid) {
					for (Queue<Double> timestampQueue : timestampQueues) {
						timestampQueue.offer(timestamp);
					}
				}
			} finally {
				SwerveDrive.odometryLock.unlock();
			}
		}
	}
}
