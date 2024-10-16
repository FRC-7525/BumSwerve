package frc.robot.subsystems.manager;

import frc.robot.Constants;
import frc.robot.pioneersLib.subsystem.Subsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;

public class Manager extends Subsystem<ManagerStates> {

    private Drive driveSubsystem;
    private Vision visionSubsystem;

    public Manager() { 
        super("Manager", ManagerStates.IDLE);

        switch (Constants.ROBOT_STATE) {
            case REAL:
                driveSubsystem = new Drive(Constants.Drive.Real.MODULE_IO, Constants.Drive.Real.GYRO_IO, false);
                visionSubsystem = new Vision(new VisionIOReal(), driveSubsystem);
                break;
            case SIM:
                driveSubsystem = new Drive(Constants.Drive.Sim.MODULE_IO, Constants.Drive.Sim.GYRO_IO, true);
                visionSubsystem = new Vision(new VisionIOSim(), driveSubsystem);
                break;
            case REPLAY:
                driveSubsystem = new Drive(Constants.Drive.Real.MODULE_IO, Constants.Drive.Real.GYRO_IO, false);
                visionSubsystem = new Vision(new VisionIO() {}, driveSubsystem);
                break;
            default:
                break;
        }
    }

    @Override
    public void runState() {
        // Call subystem periodics
        driveSubsystem.periodic();

        // TODO: Uncoment when drive is fully working
        // visionSubsystem.periodic();

        // Drive the robot
        driveSubsystem.drive(() -> Constants.CONTROLLER.getLeftX(), () -> Constants.CONTROLLER.getLeftY(), () -> Constants.CONTROLLER.getRightX(), true, false);
    }

}  
