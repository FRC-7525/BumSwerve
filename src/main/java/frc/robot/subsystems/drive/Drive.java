package frc.robot.subsystems.drive;


import com.ctre.phoenix6.SignalLogger;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.pioneersLib.bumSwerve.SwerveDrive;
import frc.robot.pioneersLib.bumSwerve.SwerveModule;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIO;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIONavX;
import frc.robot.pioneersLib.bumSwerve.Gyro.SwerveGyroIOSim;
import frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder.SwerveAbsoluteEncoderIOCANcoder;
import frc.robot.pioneersLib.bumSwerve.SwerveAbsoluteEncoder.SwerveAbsoluteEncoderIOSim;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIOKrakenSim;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIONeoSim;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIOSparkMax;
import frc.robot.pioneersLib.bumSwerve.SwerveMotor.SwerveMotorIOTalonFX;
import frc.robot.pioneersLib.subsystem.Subsystem;

public class Drive extends Subsystem<DriveStates> {
    private SwerveDrive drive;

    private final double WHEEL_RADIUS = Units.inchesToMeters(2);
    private final double TRACK_WIDTH_X = Units.inchesToMeters(25);
    private final double TRACK_WIDTH_Y = Units.inchesToMeters(25);
    private final double MAX_SPEED = Units.feetToMeters(19.5);

    private SwerveGyroIO gyroIO;
    private SwerveModule[] modules;
    private XboxController controller;

    private boolean sim;
    private SysIdRoutine sysIdDrive;
    private SysIdRoutine sysIdDriveRotation;
    private String state;
    CommandScheduler commandScheduler;

    private XboxController sysIdController;
    private XboxController secondarySysIdController;

    public Drive() {
        super("Drive", DriveStates.REGULAR);
        SignalLogger.enableAutoLogging(true);
        SignalLogger.setPath("Logs");
        SignalLogger.start();
        sim = true;
        state = "none";

        controller = new XboxController(0);
        sysIdController = new XboxController(4);
        secondarySysIdController = new XboxController(5);
        

        commandScheduler = CommandScheduler.getInstance();

        if (sim) {
            gyroIO = new SwerveGyroIOSim();
            modules = new SwerveModule[] {
                    new SwerveModule(new SwerveMotorIOKrakenSim(1, 5.357, 0.000520786),
                            new SwerveMotorIONeoSim(5, 21.4286),
                            new SwerveAbsoluteEncoderIOSim(9, 121.0), "FrontLeft"),
                    new SwerveModule(new SwerveMotorIOKrakenSim(2, 5.357, 0.000520786),
                            new SwerveMotorIONeoSim(6, 21.4286),
                            new SwerveAbsoluteEncoderIOSim(10, 11.0), "FrontRight"),
                    new SwerveModule(new SwerveMotorIOKrakenSim(3, 5.357, 0.000520786),
                            new SwerveMotorIONeoSim(7, 21.4286),
                            new SwerveAbsoluteEncoderIOSim(11, 21.0), "BackLeft"),
                    new SwerveModule(new SwerveMotorIOKrakenSim(4, 5.357, 0.000520786),
                            new SwerveMotorIONeoSim(8, 21.4286),
                            new SwerveAbsoluteEncoderIOSim(12, 1), "BackRight")
            };
        } else {
            gyroIO = new SwerveGyroIONavX(1);
            modules = new SwerveModule[] {
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(1, 5.357),
                            new SwerveMotorIOSparkMax(5, 21.4286),
                            new SwerveAbsoluteEncoderIOCANcoder(2, 315), "FrontLeft"),
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(2, 5.357),
                            new SwerveMotorIOSparkMax(6, 21.4286),
                            new SwerveAbsoluteEncoderIOCANcoder(2, 31),
                            "FrontRight"),
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(3, 5.357),
                            new SwerveMotorIOSparkMax(7, 21.4286),
                            new SwerveAbsoluteEncoderIOCANcoder(2, 315),
                            "BackLeft"),
                    new SwerveModule(
                            new SwerveMotorIOTalonFX(4, 5.357),
                            new SwerveMotorIOSparkMax(8, 21.4286),
                            new SwerveAbsoluteEncoderIOCANcoder(2, 124),
                            "BackRight")
            };
        }

        drive = new SwerveDrive(TRACK_WIDTH_X, TRACK_WIDTH_Y, modules, gyroIO, MAX_SPEED, WHEEL_RADIUS, sim);
        // TODO: Tune
        drive.configureAnglePID(0.1, 0, 0.0);
        drive.configureDrivePID(0.01, 0, 0);

        sysIdDrive =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> SignalLogger.writeString("Phoenix6/SysIdStateDrive", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runVoltDrive((voltage.in(Volts)));
                  }
                },
                null,
                this));
                
        sysIdDriveRotation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> SignalLogger.writeString("Phoenix6/SysIdStateRotation", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runVoltRotation((voltage.in(Volts)));
                  }
                },
                null,
                this));


                addRunnableTrigger(() -> commandScheduler.schedule(sysIdQuasistaticDrive(Direction.kForward)), () -> sysIdController.getXButtonPressed());
                addRunnableTrigger(() -> commandScheduler.schedule(sysIdQuasistaticDrive(Direction.kReverse)), () -> sysIdController.getYButtonPressed());
                addRunnableTrigger(() -> commandScheduler.schedule(sysIdDynamicDrive(Direction.kForward)), () -> sysIdController.getAButtonPressed());
                addRunnableTrigger(() -> commandScheduler.schedule(sysIdDynamicDrive(Direction.kReverse)), () -> sysIdController.getBButtonPressed());
        
                addRunnableTrigger(() -> commandScheduler.schedule(sysIdQuasistaticRotation(Direction.kForward)), () -> secondarySysIdController.getXButtonPressed());
                addRunnableTrigger(() -> commandScheduler.schedule(sysIdQuasistaticRotation(Direction.kReverse)), () -> secondarySysIdController.getYButtonPressed());
                addRunnableTrigger(() -> commandScheduler.schedule(sysIdDynamicRotation(Direction.kForward)), () -> secondarySysIdController.getAButtonPressed());
                addRunnableTrigger(() -> commandScheduler.schedule(sysIdDynamicRotation(Direction.kReverse)), () -> secondarySysIdController.getBButtonPressed());
        }

        public void runState() {
                drive.periodic();
                // Drive the robot
                drive.drive(() -> controller.getLeftY(), () -> controller.getLeftX(), () -> controller.getRightX(),
                        true, false);
                SignalLogger.writeString("SysIdState", state);

        }

        // command factories my favorite!!!
        public Command sysIdQuasistaticDrive(SysIdRoutine.Direction direction) {
                return sysIdDrive.quasistatic(direction);
        }

        /** Returns a command to run a dynamic test in the specified direction. */
        public Command sysIdDynamicDrive(SysIdRoutine.Direction direction) {
                return sysIdDrive.dynamic(direction);
        } 

        public Command sysIdQuasistaticRotation(SysIdRoutine.Direction direction) {
                return sysIdDriveRotation.quasistatic(direction);
        }

        public Command sysIdDynamicRotation(SysIdRoutine.Direction direction) {
                return sysIdDriveRotation.dynamic(direction);
        } 
}
