package frc.robot.subsystems.manager;

import frc.robot.pioneersLib.subsystem.Subsystem;

public class Manager extends Subsystem<ManagerStates> {

    public Manager() { 
        super("Manager", ManagerStates.IDLE);
    }

    @Override
    public void runState() {

    }

}  
