package frc.robot.subsystems.manager;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.pioneersLib.subsystem.Subsystem;
import frc.robot.subsystems.drive.Drive;

public class Manager extends Subsystem<ManagerStates> {

    private Drive driveSubsystem;
    private XboxController controller;

    public Manager() { 
        super("Manager", ManagerStates.IDLE);

        switch (Constants.ROBOT_STATE) {
            case REAL:
                driveSubsystem = new Drive(Constants.Drive.Real.MODULE_IO, Constants.Drive.Real.GYRO_IO);
                break;
            case SIM:
                driveSubsystem = new Drive(Constants.Drive.Sim.MODULE_IO, Constants.Drive.Sim.GYRO_IO);
                break;
            case REPLAY:
                driveSubsystem = new Drive(Constants.Drive.Real.MODULE_IO, Constants.Drive.Real.GYRO_IO);
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
        driveSubsystem.drive(() -> controller.getLeftX(), () -> controller.getLeftY(), () -> controller.getRightX(), true, false);
    }

}  
