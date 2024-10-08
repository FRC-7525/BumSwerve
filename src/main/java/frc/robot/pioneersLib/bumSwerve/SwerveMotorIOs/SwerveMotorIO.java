package frc.robot.pioneersLib.bumSwerve.SwerveMotorIOs;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveMotorIO {

    //Defines all the inputs that need to be logged from each motor
    @AutoLog
    public static class SwerveMotorIOInputs {
        public Rotation2d motorPosition = new Rotation2d();
        public double motorVelocityRPS = 0.0;
        public double[] motorCurrentAmps = new double[] {};

        public double[] odometryTimestamps = new double[] {};
        public Rotation2d[] odometryMotorPositions = new Rotation2d[] {};
    }

    //Defines all the outputs that need to be logged from each motor
    public static class SwerveMotorIOOutputs {
        public double motorAppliedVolts = 0.0;
    }

    /** 
     * Updates the set of loggable inputs.
     * @param inputs
     */
    public default void updateInputs(SwerveMotorIOInputs inputs) {}


    /**
     * Updates the set of loggable outputs
     * @param outputs
     */
    public default void updateOutputs(SwerveMotorIOOutputs outputs) {}

    /**
     * Run the motor at the specified voltage
     * @param volts
     */
    public default void setVoltage(double volts) {}

    /**
     * Use PID to move the motor to the specified setpoint
     * <br></br>
     * Only use with turn motors
     * @param setpoint Position in degrees
     */
    public default void setPosition(double setpoint) {}

    /**
     * Use FeedForward/Feedback PID to get motor to specified velocity
     * <br></br>
     * Only use with drive motors
     * @param speedpoint Velocity in RPS
     */
    public default void setVelocity(double speedpoint) {}

    /**
     * Enable or disable brake mode on the motor
     * @param enable
     */
    public default void setBrakeMode(boolean enable) {}

    /**
     * Configure the PID values for the controller used for this motor
     * @param kP
     * @param kI
     * @param kD
     */
    public default void configurePID(double kP, double kI, double kD) {}

    /**
     * Sets a feed forward controller based on several real-world parameters. Not used in SIM
     * <br></br>
     * Controller is used only for drive
     * @param optimalVoltage
     * @param maxLinearSpeed In meters
     * @param wheelGripCoefficientOfFriction
     */
    public default void setFeedForward(double optimalVoltage, double maxLinearSpeed, double wheelGripCoefficientOfFriction) {}

    /**
     * Calculates the max acceleration of the wheel given the coefficient of friction and using gravity
     * @param cof Coefficient of friction
     * @return Max acceleration of the wheel
     */
    public default double calculateMaxAcceleration(double cof) {
        return 0.0;
    }
}
