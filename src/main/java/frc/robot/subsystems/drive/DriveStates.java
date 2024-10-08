package frc.robot.subsystems.drive;

import frc.robot.pioneersLib.subsystem.SubsystemStates;

public enum DriveStates implements SubsystemStates{
    REGULAR("Regular Drive");

    DriveStates(String stateString) {
        this.stateString = stateString;
    }

    String stateString;

    public String getStateString() {
        return stateString;
    }
}
