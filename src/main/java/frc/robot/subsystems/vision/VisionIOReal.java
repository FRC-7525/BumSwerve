package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

public class VisionIOReal implements VisionIO {

    private PhotonCamera sideCamera;
    private PhotonCamera frontCamera;
    private PhotonPoseEstimator sideEstimator;
    private PhotonPoseEstimator frontEstimator;
    private Debouncer sideDebouncer;
    private Debouncer frontDebouncer;
    
    public VisionIOReal() {


        sideCamera = new PhotonCamera("Side Camera");
        frontCamera = new PhotonCamera("Front Camera");

        // Pose estimators :/
        frontEstimator = new PhotonPoseEstimator(Constants.Vision.APRIL_TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera, Constants.Vision.ROBOT_TO_FRONT_CAMERA);
        sideEstimator = new PhotonPoseEstimator(Constants.Vision.APRIL_TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, sideCamera, Constants.Vision.ROBOT_TO_SIDE_CAMERA);
        sideDebouncer = new Debouncer(0.5, DebounceType.kFalling);
        frontDebouncer = new Debouncer(0.5, DebounceType.kFalling);
    }

    @Override
    public void updateInptus(VisionIOInputs inputs) {
        Optional<EstimatedRobotPose> sidePose = sideEstimator.update();
        Optional<EstimatedRobotPose> frontPose = frontEstimator.update();

        inputs.hasSideVision = sideDebouncer.calculate(sidePose.isPresent());
        inputs.hasFrontVision = frontDebouncer.calculate(frontPose.isPresent());
        inputs.sideCameraConnected = sideCamera.isConnected();
        inputs.frontCameraConnected = frontCamera.isConnected();
        inputs.sideTargetCount = sidePose.get().targetsUsed.size();
        inputs.frontTargetCount = frontPose.get().targetsUsed.size();
        if (inputs.hasSideVision) inputs.sideVisionPose = sidePose.get().estimatedPose.toPose2d();
        if (inputs.hasFrontVision) inputs.frontVisionPose = frontPose.get().estimatedPose.toPose2d();
    }

    @Override
    public void updateRobotPose(Pose2d pose) {
        // U dont need robot pose for real life
        return;
    } 

    @Override
    public void setStrategy(PoseStrategy strategy) {
        if (strategy != frontEstimator.getPrimaryStrategy()) {
            frontEstimator.setPrimaryStrategy(strategy);
            sideEstimator.setPrimaryStrategy(strategy);
        }
    }

    // Not just returning a pose3d bc timestamps needed for main pose estimation & easier to handle optional logic in vision.java
    @Override
    public Optional<EstimatedRobotPose> getSidePoseEstimation() {
        Optional<EstimatedRobotPose> pose = sideEstimator.update();
        return pose;
    }

    @Override
    public Optional<EstimatedRobotPose> getFrontPoseEstimation() {
        Optional<EstimatedRobotPose> pose = frontEstimator.update();
        return pose;
    }
}
