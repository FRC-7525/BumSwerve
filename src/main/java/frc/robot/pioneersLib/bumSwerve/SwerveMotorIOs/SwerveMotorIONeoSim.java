package frc.robot.pioneersLib.bumSwerve.SwerveMotorIOs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;

public class SwerveMotorIONeoSim implements SwerveMotorIO {
    private REVPhysicsSim revSim;
    private CANSparkMax dummySpark;
    private double gearRatio;
    private boolean isDrive;

    public SwerveMotorIONeoSim(int id, boolean isDrive, double gearRatio) {
        revSim = REVPhysicsSim.getInstance();
        dummySpark = new CANSparkMax(id, MotorType.kBrushless);

        revSim.addSparkMax(dummySpark, DCMotor.getNEO(1));
    }
}
