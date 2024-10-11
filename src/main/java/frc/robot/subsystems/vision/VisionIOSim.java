package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class VisionIOSim implements VisionIO {

    private VisionSystemSim visionSim;
    private AprilTagFieldLayout fieldLayout;
    private SimCameraProperties sideCameraProperties;
    private SimCameraProperties frontCameraProperties;
    private PhotonCameraSim sideCamera;
    private PhotonCameraSim frontCamera;
    private PhotonPoseEstimator sideEstimator;
    private PhotonPoseEstimator frontEstimator;
    private Debouncer sideDebouncer;
    private Debouncer frontDebouncer;
    private Pose2d robotPose;
    
    public VisionIOSim() {
        visionSim = new VisionSystemSim("Vision");
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        sideCameraProperties = new SimCameraProperties();
        frontCameraProperties = new SimCameraProperties();

        // TODO: Tune to accurate values & put in constants ig
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        sideCameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        frontCameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        sideCameraProperties.setCalibError(0.25, 0.08);
        frontCameraProperties.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        sideCameraProperties.setFPS(20);
        frontCameraProperties.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        sideCameraProperties.setAvgLatencyMs(35);
        sideCameraProperties.setLatencyStdDevMs(5);
        frontCameraProperties.setAvgLatencyMs(35);
        frontCameraProperties.setLatencyStdDevMs(5);

        sideCamera = new PhotonCameraSim(new PhotonCamera("Side Camera"), sideCameraProperties);
        frontCamera = new PhotonCameraSim(new PhotonCamera("Front Camera"), frontCameraProperties);

        visionSim.addAprilTags(fieldLayout);
        visionSim.addCamera(sideCamera, Constants.Vision.ROBOT_TO_SIDE_CAMERA);
        visionSim.addCamera(frontCamera, Constants.Vision.ROBOT_TO_FRONT_CAMERA);

        // Puts a camera stream onto nt4
        frontCamera.enableRawStream(true);
        frontCamera.enableProcessedStream(true);
        sideCamera.enableRawStream(true);
        sideCamera.enableProcessedStream(true);
        // Disable this if ur laptop is bad (makes the camera stream easy to understanda)
        frontCamera.enableDrawWireframe(true);
        sideCamera.enableDrawWireframe(true);

        robotPose = new Pose2d();
        // Pose estimators :/
        frontEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera.getCamera(), Constants.Vision.ROBOT_TO_FRONT_CAMERA);
        sideEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, sideCamera.getCamera(), Constants.Vision.ROBOT_TO_SIDE_CAMERA);
        sideDebouncer = new Debouncer(0.5, DebounceType.kFalling);
        frontDebouncer = new Debouncer(0.5, DebounceType.kFalling);

        SmartDashboard.putData("Vision Debug Field", visionSim.getDebugField());
    }

    @Override
    public void updateInptus(VisionIOInputs inputs) {
        // TODO: Don't call .update so much :(
        inputs.hasSideVision = sideDebouncer.calculate(sideEstimator.update().isPresent());
        inputs.hasFrontVision = frontDebouncer.calculate(frontEstimator.update().isPresent());
        inputs.sideCameraConnected = sideCamera.getCamera().isConnected();
        inputs.frontCameraConnected = frontCamera.getCamera().isConnected();
        inputs.sideTargetCount = sideEstimator.update().get().targetsUsed.size();
        inputs.frontTargetCount = frontEstimator.update().get().targetsUsed.size();
    }

    @Override
    public void updateRobotPose(Pose2d pose) {
        robotPose = pose;
        visionSim.update(robotPose);
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
