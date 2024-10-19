package frc.robot.subsystems.manager;

import frc.robot.pioneersLib.subsystem.SubsystemStates;

public enum ManagerStates implements SubsystemStates{
    IDLE("Idle");

    ManagerStates(String stateString) {
        this.stateString = stateString;

    }

    String stateString;

    @Override
    public String getStateString() {
        return stateString;
    }
}
