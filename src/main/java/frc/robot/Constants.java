package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class Constants {
    enum RobotState {
        SIM, REAL, REPLAY
    }

    public static final RobotState robotState = RobotState.SIM;

    public static final class Vision {
        // Robot to cam 
        public static final Translation3d ROBOT_TO_SIDE_CAMERA_TRALSLATION = new Translation3d(Units.inchesToMeters(-7.19), Units.inchesToMeters(11), Units.inchesToMeters(15.25));
        public static final Rotation3d ROBOT_TO_SIDE_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-20), Math.toRadians(90));
        public static final Transform3d ROBOT_TO_SIDE_CAMERA = new Transform3d(ROBOT_TO_SIDE_CAMERA_TRALSLATION, ROBOT_TO_SIDE_CAMERA_ROTATION);
        public static final Translation3d ROBOT_TO_FRONT_CAMERA_TRANSLATION = new Translation3d(Units.inchesToMeters(-14.25), 0, Units.inchesToMeters(6));
        public static final Rotation3d ROBOT_TO_FRONT_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-67), Math.toRadians(180));
        public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(ROBOT_TO_FRONT_CAMERA_TRANSLATION, ROBOT_TO_FRONT_CAMERA_ROTATION);

        // Camera Quality
        public enum CameraResolution {
            HIGH_RES,
            NORMAL
        }

        // TODO: What camera resolutions actually are these?
        public static final CameraResolution SIDE_RESOLUTION = CameraResolution.HIGH_RES;
        public static final CameraResolution FRONT_RESOLUTION = CameraResolution.HIGH_RES;

        // STDEV Pre-calculated
        public static final Matrix<N3, N1> highResSingleTagStdDev = VecBuilder.fill(0.4, 0.4, Double.MAX_VALUE);
	    public static final Matrix<N3, N1> normalSingleTagStdDev = VecBuilder.fill(0.8, 0.8, Double.MAX_VALUE);
	    public static final Matrix<N3, N1> highResMultiTagStdDev = VecBuilder.fill(0.2, 0.2, 3);
	    public static final Matrix<N3, N1> normalMultiTagStdDev = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);

        // Other
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    }
}
