package frc.robot.vision;

import java.io.IOException;
import java.lang.annotation.Target;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.drive.Drive;
import frc.robot.test.console_output.Output;

public class Vision {
    private static final String CAMERA_NAME = "photoncamera";
    /*
     * Assume that you're looking at the robot from above it. In our code we treat
     * the robot like a single point in an XY-Plane. Where the front of the robot is
     * the
     * positive X, and where the left side of the robot is the negative Y.
     */
    // How far foward/backward the camera is from robot center.
    private static final double CAMERA_X_FROM_ROBOT_CENTER = 0.193;
    // How far left/right the camera is from robot center.
    private static final double CAMERA_Y_FROM_ROBOT_CENTER = 0.2794;
    // How far up/down the camera is from center if we look at robot from side in 3D
    // space.
    private static final double CAMERA_Z_FROM_ROBOT_CENTER = 0.375;

    private static final double CAMERA_ROLL = 0;
    private static final double CAMERA_PITCH = 0;
    private static final double CAMERA_YAW = Math.toRadians(-10.5);

    private static boolean hasTarget;
    private static boolean isServerConnected;

    private static Transform3d cameraPos = new Transform3d(
            new Translation3d(CAMERA_X_FROM_ROBOT_CENTER, CAMERA_Y_FROM_ROBOT_CENTER, CAMERA_Z_FROM_ROBOT_CENTER),
            new Rotation3d(CAMERA_ROLL, CAMERA_PITCH, CAMERA_YAW));

    private static AprilTagFieldLayout layout;
    static {
        try {layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException exception) {Output.warning("Cannot Load Apriltag. Vision Results Cannot Be Used.");}
    };

    static final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);
    private static final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
            layout,
            PoseStrategy.LOWEST_AMBIGUITY,
            camera,
            cameraPos);

    /**
     * {@summary} The angle of the apriltag from the robot based off odometry.
     * 
     * @param apriltagID
     * @param odometry
     * @return Angle in degrees to apriltag from robot.
     * @throws Exception
     * @throws IOException
     */
    public static double getAngleToApriltag(int apriltagID, SwerveDrivePoseEstimator odometry)
            throws Exception, IOException {
        Pose2d robotPose = odometry.getEstimatedPosition();
        Pose2d apriltagPose = layout.getTagPose(apriltagID).isPresent()
                ? (layout.getTagPose(apriltagID).get().toPose2d())
                : null;

        if (apriltagPose == null) {
            Output.warning("Tag Couldn't Be Located. Ensure Apriltag ID is correct. ");
        }
        // System.out.println("Current Robot Angle:" + robotPose.relativeTo(apriltagPose).getRotation().getDegrees());
        return robotPose.relativeTo(apriltagPose).getRotation().getDegrees();
    }

    /**
     * {@summary} Uses Translations to compare the two Pose2d of the apriltag and
     * robot and then find distance between them.
     * 
     * @param apriltagID
     * @param odometry
     * @return Distance to apriltag given an ID.
     * @throws Exception
     * @throws IOException
     */
    public static double getDistanceToApriltag(int apriltagID, SwerveDrivePoseEstimator odometry)
            throws Exception, IOException {
        Pose2d robotPose = odometry.getEstimatedPosition();
        Pose2d apriltagPose = layout.getTagPose(apriltagID).isPresent()
                ? (layout.getTagPose(apriltagID).get().toPose2d())
                : null;

        // Null check to ensure that we aren't working with a null value.
        if (apriltagPose == null) {
            System.out.println("Tag couldn't be found. Please ensure that ID for apriltag is correct.");
            throw new Exception("Tag couldn't be found. Please ensure that ID for apriltag is correct.");
        }
        // Pretty sure this returns the distance betweeen the robot and the apriltag.
        return robotPose.relativeTo(apriltagPose).getTranslation().getDistance(apriltagPose.getTranslation());
    }

    /**
     * {@summary} Takes in the pose estimator used for odometry then if there aren't
     * any Apriltags detected just returns it, but if there are some then adjusts
     * the pose estimation to account for the values it gets from them.
     * 
     * @param poseEstimator
     * @return SwerveDrivePoseEstimator from WPILIB
     */
    public static SwerveDrivePoseEstimator getVisionEstimatedRobotPose(SwerveDrivePoseEstimator poseEstimator) {
        PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
        if (target != null) {
            hasTarget = true;
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                        layout.getTagPose(target.getFiducialId()).get(), cameraPos);
            poseEstimator.addVisionMeasurement(robotPose.toPose2d(), Timer.getFPGATimestamp());
            System.out.println("[INFO]: Robot Pose Updated From Vision " + robotPose);
            return poseEstimator;
        } else {
            hasTarget = false;
            //System.out.println("[WARNING]: Robot Cannot See Apriltag");
            return poseEstimator;
        }
    }

    /**
     * {@summary} Should be called whenever intializing the odometry system. Uses Apriltag to localize the
     * robot on the field. <Strong>Note: Needs to be facing an Apriltag or exception thrown.</Strong>
     * @return Pose of robot in three dimensions. 
     * @throws NullPointerException Apriltag cannot be found. 
     */
    public static Pose3d getRobotInitialPose() {
        try {
        PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                        layout.getTagPose(target.getFiducialId()).get(), cameraPos);
        return robotPose;
        }
        catch(Exception exception){
            Output.error("Cannot Determine Initial Pose of Robot. Default 0 Pose Set.");
            return new Pose3d(0, 0, 0, new Rotation3d());
        }
    }

    /**
     * {@summary} Returns an object representing the apriltag location on the game
     * field used for Pose estimation. If path cannot be found returns null.
     * 
     * @throws IOException
     * @return AprilTag Field Layout
     */
    private static Optional<EstimatedRobotPose> getEstimatedRobotGlobalPose(Pose2d previousEstimatedRobotPose) {
        poseEstimator.setReferencePose(previousEstimatedRobotPose);
        return poseEstimator.update();
    }

    public static boolean hasTarget(){
        return hasTarget;
    }
    public static boolean isServerConnected(){return camera != null;}
}
