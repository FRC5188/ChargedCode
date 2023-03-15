package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TrajectoryBuilder {
    private PathConstraints _pathConstraints;
    private List<PathPoint> _pathPoints = new ArrayList<>();

    /** Should only be called once and sets the constraints for velocity and acceleration for the entire path.  */
    public TrajectoryBuilder addConstraints(double maxVelocity, double maxAcceleration){
        this._pathConstraints = new PathConstraints(maxVelocity, maxAcceleration);
        return this;
    }
    /** Called whenever you want to add another point to the path.  */
    public TrajectoryBuilder addPathPoint(double x, double y, double heading){
        this._pathPoints.add(new PathPoint(new Translation2d(x, y), new Rotation2d(heading)));
        return this;
    }

    public TrajectoryBuilder addPathPoint(PathPoint pathPoint){
        this._pathPoints.add(pathPoint);
        return this;
    }

    /** <STRONG>Must be called as the first operation of the builder</STRONG>. It sets that start position to the current positon of the robot that is passed in. */
    public TrajectoryBuilder setStartPositionAsCurrent(Pose2d currentRobotPose){
        this._pathPoints.add(new PathPoint(currentRobotPose.getTranslation(), currentRobotPose.getRotation()));
        return this;
    }
    
    /** Called whenver you've finished designing your current path. It returns the trajectory itself. */
    public PathPlannerTrajectory buildTrajectory(){
        return PathPlanner.generatePath(this._pathConstraints, this._pathPoints);
    }

    /**  When you need to see information about the class call this method to print important information to the console.  */
    public void debug(){
        System.out.printf("Current Constraints: Maximum Velocity = %d Maximum Acceleration = %d \n", this._pathConstraints.maxVelocity, this._pathConstraints.maxVelocity);
        for(PathPoint pathPoint: this._pathPoints){
            System.out.printf("PathPoint Information: %s \n", pathPoint.toString());
        }
    }
}
