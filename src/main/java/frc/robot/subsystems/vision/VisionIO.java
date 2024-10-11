package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {

    @AutoLog
    public class VisionIOInputs {

    }

    public default void updateInptus(VisionIOInputs inputs) {}
    
    public default void updateRobotPose(Pose2d robotPose) {}

    public default void setStrategy(PoseStrategy strategy) {}

    public default Optional<EstimatedRobotPose> getFrontPoseEstimation() {
        return Optional.empty();
    }
    public default Optional<EstimatedRobotPose> getSidePoseEstimation() {
        return Optional.empty();
    }

}
