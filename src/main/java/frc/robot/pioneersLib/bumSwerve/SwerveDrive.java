package frc.robot.pioneersLib.bumSwerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class SwerveDrive {
    static final Lock odometryLock = new ReentrantLock();

    public SwerveDrive() {
    }
}
