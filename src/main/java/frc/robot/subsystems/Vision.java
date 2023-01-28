package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants.Field;
import frc.robot.Constants.OperatorConstants.Vision.Camera;
import frc.robot.Constants.OperatorConstants.Vision.PhotonVision;

public class Vision extends SubsystemBase {    
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;

    public Vision(){
        this.photonCamera = new PhotonCamera(PhotonVision.PHOTON_CAMERA_NAME);
        this.photonPoseEstimator = new PhotonPoseEstimator(getApriltagFieldLayout(), PoseStrategy.AVERAGE_BEST_TARGETS, photonCamera, Camera.DIFFERENCE_BETWEEN_ROBOT_CAMERA);
    }

    /**
     * {@summary} Takes in the pose estimator used for odometry then if there aren't any Apriltags detected just returns it, but if there are some then adjusts
     * the pose estimation to account for the values it gets from them. 
     * @param poseEstimator
     * @return SwerveDrivePoseEstimator from WPILIB
     */
    public SwerveDrivePoseEstimator getVisionEstimatedRobotPose(SwerveDrivePoseEstimator poseEstimator){
        Optional<EstimatedRobotPose> photonResults = getEstimatedRobotGlobalPose(poseEstimator.getEstimatedPosition());
        if(photonResults.isEmpty()){
            return poseEstimator;
        }
        else {
            EstimatedRobotPose robotEstimatedPose = photonResults.get();
            poseEstimator.addVisionMeasurement(robotEstimatedPose.estimatedPose.toPose2d(), robotEstimatedPose.timestampSeconds);
            return poseEstimator;
        }
    }


    private Optional<EstimatedRobotPose> getEstimatedRobotGlobalPose(Pose2d previousEstimatedRobotPose){
        photonPoseEstimator.setReferencePose(previousEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    /**
     * {@summary} Returns an object representing the apriltag location on the game field used for Pose estimation. If path cannot be found returns null. 
     * @throws IOException
     * @return AprilTag Field Layout
     */
    private static AprilTagFieldLayout getApriltagFieldLayout()
    {
        try {
            return new AprilTagFieldLayout(Field.AprilTag.APRIL_TAG_FIELD_MAP_PATH);
        } 
        catch (IOException ioException) {
            System.out.println("ERROR: Cannot open file for Apriltag Field Map. File path may be incorrect.");
            return null;
        }
    }
}
