package frc.robot.subsystems.manager;

import frc.robot.Constants;
import frc.robot.pioneersLib.subsystem.Subsystem;
import frc.robot.subsystems.drive.Drive;

public class Manager extends Subsystem<ManagerStates> {

    private Drive driveSubsystem;

    public Manager() { 
        super("Manager", ManagerStates.IDLE);

        switch (Constants.ROBOT_STATE) {
            case REAL:
                driveSubsystem = new Drive(Constants.Drive.Real.MODULE_IO, Constants.Drive.Real.GYRO_IO, false);
                break;
            case SIM:
                driveSubsystem = new Drive(Constants.Drive.Sim.MODULE_IO, Constants.Drive.Sim.GYRO_IO, true);
                break;
            case REPLAY:
                driveSubsystem = new Drive(Constants.Drive.Real.MODULE_IO, Constants.Drive.Real.GYRO_IO, false);
                break;
            default:
                break;
        }
    }

    @Override
    public void runState() {
        // Call subystem periodics
        driveSubsystem.periodic();

        // Drive the robot
        driveSubsystem.drive(() -> Constants.CONTROLLER.getLeftX(), () -> Constants.CONTROLLER.getLeftY(), () -> Constants.CONTROLLER.getRightX(), true, false);
    }

}  
