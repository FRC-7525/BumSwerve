package frc.robot.pioneersLib.subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

public abstract class Subsystem<StateType extends SubsystemStates> extends SubsystemBase {

	private Map<StateType, ArrayList<Trigger<StateType>>> triggerMap = new HashMap<
		StateType,
		ArrayList<Trigger<StateType>>
	>();

	private List<RunnableTrigger> runnableTriggerMap = new ArrayList<>();

	private StateType state = null;
	private Timer stateTimer = new Timer();
	private String subsystemName;

	public Subsystem(String subsystemName, StateType defaultState) {
		if (defaultState == null) {
			throw new RuntimeException("Default state cannot be null!");
		}
		this.subsystemName = subsystemName;
		this.state = defaultState;

		stateTimer.start();
	}

	// State operation
	public void periodic() {
		// Logger.recordOutput(subsystemName + "/state", state.getStateString());

		runState();

		checkTriggers();
		checkRunnableTriggers();
	}

	protected abstract void runState();

	// SmartDashboard utils
	protected void putSmartDashboard(String key, String value) {
		SmartDashboard.putString("[" + subsystemName + "] " + key, value);
	}

	protected void putSmartDashboard(String key, double value) {
		SmartDashboard.putNumber("[" + subsystemName + "] " + key, value);
	}

	// Triggers for state transitions
	protected void addTrigger(StateType startType, StateType endType, BooleanSupplier check) {
		if (triggerMap.get(startType) == null) {
			triggerMap.put(startType, new ArrayList<Trigger<StateType>>());
		}
		triggerMap.get(startType).add(new Trigger<StateType>(check, endType));
	}

	private void checkTriggers() {
		List<Trigger<StateType>> triggers = triggerMap.get(state);
		if (triggers == null) return;
		for (var trigger : triggers) {
			if (trigger.isTriggered()) {
				setState(trigger.getResultState());
				return;
			}
		}
	}

	protected void addRunnableTrigger(Runnable runnable, BooleanSupplier check) {
		runnableTriggerMap.add(new RunnableTrigger(check, runnable));
	}

	private void checkRunnableTriggers() {
		if (triggerMap == null) return;
		for (var trigger : runnableTriggerMap) {
			if (trigger.isTriggered()) {
				trigger.run();
				return;
			}
		}
	}

	// Other utilities
	public StateType getState() {
		return state;
	}

	public void setState(StateType state) {
		if (this.state != state) stateTimer.reset();
		this.state = state;
	}

	/**
	 * Gets amount of time the state machine has been in the current state.
	 *
	 * @return time in seconds.
	 */
	protected double getStateTime() {
		return stateTimer.get();
	}
}
