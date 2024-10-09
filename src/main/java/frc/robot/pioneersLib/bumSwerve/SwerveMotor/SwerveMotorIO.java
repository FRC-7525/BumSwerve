package frc.robot.pioneersLib.bumSwerve.SwerveMotor;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveMotorIO {

    //Defines all the inputs that need to be logged from each motor
    @AutoLog
    public static class SwerveMotorIOInputs {
        public Rotation2d motorPosition = new Rotation2d();
        public double motorVelocityRPS = 0.0;
        public double[] motorCurrentAmps = new double[] {};

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryMotorAccumulatedPosition = new double[] {};
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
     * Returns the current angle of the motor as a Rotation2d
     */
    public default Rotation2d getAngle() {
        return new Rotation2d();
    }

    /**
     * Returns the current position error of the motor from the feedback controller
     */
    public default double getPositionError() {
        return 0.0;
    }

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
     * 
     * @param kS
     * @param kV
     * @param kA
     */
    public default void configureFF(double kS, double kV, double kA) {}

    // TODO: Make this a function in module.java as it should be used in sim & does not need to be isolated to motors
    /**
     * Sets a feed forward controller based on several real-world parameters. Not used in SIM
     * <br></br>
     * Controller is used only for drive
     * @param optimalVoltage
     * @param maxLinearSpeed In meters
     * @param wheelGripCoefficientOfFriction
     */
    // public default void setFeedForward(double optimalVoltage, double maxLinearSpeed, double wheelGripCoefficientOfFriction) {}

    //TODO: Put this in module, not motor (unless there's a good reason)
    /**
     * Calculates the max acceleration of the wheel given the coefficient of friction and using gravity
     * @param cof Coefficient of friction
     * @return Max acceleration of the wheel
     */
    public default double calculateMaxAcceleration(double cof) {
        return 0.0;
    }
}
