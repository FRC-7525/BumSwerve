package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.pioneersLib.subsystem.Subsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Constants;
import frc.robot.Constants.Vision.*;

public class Vision extends Subsystem<VisionStates> {

    private VisionIO io;
    private Drive drive;

    public Vision(VisionIO io, Drive drive) {
        super("Vision", VisionStates.ON);

        this.io = io;
        this.drive = drive;
    }

    @Override
    public void runState() {
        if (getState().getVisionEnabled()) {
            io.setStrategy(getState().getStrategy());
            io.updateRobotPose(drive.getPose());
    
            Optional<EstimatedRobotPose> frontPose = io.getFrontPoseEstimation();
            if (io.getFrontPoseEstimation().isPresent()) {
                drive.addVisionMeasurment(frontPose.get().estimatedPose.toPose2d(), frontPose.get().timestampSeconds,
                        getEstimationStdDevs(frontPose.get(), Constants.Vision.FRONT_RESOLUTION));
            }
    
            Optional<EstimatedRobotPose> sidePose = io.getSidePoseEstimation();
            if (sidePose.isPresent()) {
                drive.addVisionMeasurment(
                        sidePose.get().estimatedPose.toPose2d(),
                        sidePose.get().timestampSeconds,
                        getEstimationStdDevs(sidePose.get(), Constants.Vision.SIDE_RESOLUTION));
            }
        }
    }

    // Thanks AEMBOT!
    // TODO: Put a lot of these values in constants or throw this function in a
    // vision util class in our team lib
    /**
     * The standard deviations of the estimated poses from vision cameras, for use
     * with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(
            EstimatedRobotPose estimatedPose, CameraResolution resolution) {
        var estStdDevs = switch (resolution) {
            case HIGH_RES -> Constants.Vision.highResSingleTagStdDev;
            case NORMAL -> Constants.Vision.normalSingleTagStdDev;
        };
        var targets = estimatedPose.targetsUsed;
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = Constants.Vision.APRIL_TAG_FIELD_LAYOUT.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose
                    .get()
                    .toPose2d()
                    .minus(estimatedPose.estimatedPose.toPose2d())
                    .getTranslation()
                    .getNorm();
        }

        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;

        // Decrease std devs if multiple targets are visible
        if (numTags > 1
                && avgDist > switch (resolution) {
                    case HIGH_RES -> 8;
                    case NORMAL -> 5;
                }) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = switch (resolution) {
                case HIGH_RES -> Constants.Vision.highResMultiTagStdDev;
                case NORMAL -> Constants.Vision.normalMultiTagStdDev;
            };
        }
        // Increase std devs based on (average) distance
        if (numTags == 1
                && avgDist > switch (resolution) {
                    case HIGH_RES -> 6;
                    case NORMAL -> 4;
                }) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 20));
        }

        return estStdDevs;
    }
}
