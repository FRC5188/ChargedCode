package frc.robot.autonomous;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;

import javax.print.attribute.standard.Fidelity;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;

public abstract class Autonomous {
    public enum FIELD_POSITIONS {
        // Single Substration Station From Score Table Perspective
        SINGLE_SUBSTRATION_LEFT,
        SINGLE_SUBSTRATION_RIGHT,

        // Double Substation Station From Score Table Perspective
        DOUBLE_SUBSTRATION_LEFT,
        DOUBLE_SUBSTRATION_RIGHT, 

        // Charge Station From Score Table Perspective
        CHARGE_STATION_LEFT,
        CHARGE_STATION_RIGHT,

        // Game Pieces On Left Side From Score Table Perspective
        CLOSEST_SCORE_TABLE_GAME_PIECE_LEFT,
        SECOND_CLOSEST_SCORE_TABLE_GAME_PIECE_LEFT,
        THIRD_CLOSEST_SCORE_TABLE_GAME_PIECE_LEFT,
        FOURTH_CLOSEST_SCORE_TABLE_GAME_PIECE_LEFT,

        // Game Pieces On Right Side From Score Table Perspective
        CLOSEST_SCORE_TABLE_GAME_PIECE_RIGHT,
        SECOND_CLOSEST_SCORE_TABLE_GAME_PIECE_RIGHT,
        THIRD_CLOSEST_SCORE_TABLE_GAME_PIECE_RIGHT,
        FOURTH_CLOSEST_SCORE_TABLE_GAME_PIECE_RIGHT,

        // Left Side Grid For Cones From Score Table Perspective
        LEFT_SIDE_GRID_CONE_CLOSEST,
        LEFT_SIDE_GRID_CONE_SECOND_CLOSEST,
        LEFT_SIDE_GRID_CONE_THIRD_CLOSEST,
        LEFT_SIDE_GRID_CONE_FOURTH_CLOSEST,
        LEFT_SIDE_GRID_CONE_FIFTH_CLOSEST,
        LEFT_SIDE_GRID_CONE_SIXTH_CLOSEST,

        // Left Side Grid For Cubes From Score Table Perspective
        LEFT_SIDE_GRID_CUBE_CLOSEST,
        LEFT_SIDE_GRID_CUBE_SECOND_CLOSEST,
        LEFT_SIDE_GRID_CUBE_THIRD_CLOSEST,

        // Right Side Grid For Cones From Score Table Perspective
        RIGHT_SIDE_GRID_CONE_CLOSEST,
        RIGHT_SIDE_GRID_CONE_SECOND_CLOSEST,
        RIGHT_SIDE_GRID_CONE_THIRD_CLOSEST,
        RIGHT_SIDE_GRID_CONE_FOURTH_CLOSEST,
        RIGHT_SIDE_GRID_CONE_FIFTH_CLOSEST,
        RIGHT_SIDE_GRID_CONE_SIXTH_CLOSEST,

        // Right Side Grid For Cubes From Score Table Perspective
        RIGHT_SIDE_GRID_CUBE_CLOSEST,
        RIGHT_SIDE_GRID_CUBE_SECOND_CLOSEST,
        RIGHT_SIDE_GRID_CUBE_THIRD_CLOSEST,

        //Left side posititions for intermediate auto start positions
        LEFT_SIDE_INTERMEDIATE_AUTO_CLOSEST,
        LEFT_SIDE_INTERMEDIATE_AUTO_SECOND_CLOSEST,
        LEFT_SIDE_INTERMEDIATE_AUTO_THIRD_CLOSEST,

        //Right side posititions for intermediate auto start positions
        RIGHT_SIDE_INTERMEDIATE_AUTO_CLOSEST,
        RIGHT_SIDE_INTERMEDIATE_AUTO_SECOND_CLOSEST,
        RIGHT_SIDE_INTERMEDIATE_AUTO_THIRD_CLOSEST,
    }
    /** Assocaites each positon that robot could be at with a PathPoint used to create on-the-fly autonomous. */
    private static final HashMap<FIELD_POSITIONS, PathPoint> fieldPositionsCoordinateMap = new HashMap<FIELD_POSITIONS, PathPoint>(){{
    // Creates the map for the field positions. 
    
        
    //this(0.0, 0.0);
    put(FIELD_POSITIONS.SINGLE_SUBSTRATION_LEFT, new PathPoint(new Translation2d(2.6, 7.0), new Rotation2d(90)));
    put(FIELD_POSITIONS.SINGLE_SUBSTRATION_RIGHT, new PathPoint(new Translation2d(13.8, 7), new Rotation2d(90)));
    put(FIELD_POSITIONS.DOUBLE_SUBSTRATION_LEFT, new PathPoint(new Translation2d(1.2, 7.4), new Rotation2d(180)));
    put(FIELD_POSITIONS.DOUBLE_SUBSTRATION_RIGHT, new PathPoint(new Translation2d(15.3, 7.3), new Rotation2d(0)));
    put(FIELD_POSITIONS.CHARGE_STATION_LEFT, new PathPoint(new Translation2d(3.88, 2.75), new Rotation2d(0)));
    put(FIELD_POSITIONS.CHARGE_STATION_RIGHT, new PathPoint(new Translation2d(12.65, 2.75), new Rotation2d(0)));
    put(FIELD_POSITIONS.CLOSEST_SCORE_TABLE_GAME_PIECE_LEFT, new PathPoint(new Translation2d(6.15, 0.9), new Rotation2d(0)));
    put(FIELD_POSITIONS.SECOND_CLOSEST_SCORE_TABLE_GAME_PIECE_LEFT, new PathPoint(new Translation2d(6.15, 2.15), new Rotation2d(0)));
    put(FIELD_POSITIONS.THIRD_CLOSEST_SCORE_TABLE_GAME_PIECE_LEFT, new PathPoint(new Translation2d(6.15, 3.37), new Rotation2d(0)));
    put(FIELD_POSITIONS.FOURTH_CLOSEST_SCORE_TABLE_GAME_PIECE_LEFT, new PathPoint(new Translation2d(6.15, 4.59), new Rotation2d(0)));
    put(FIELD_POSITIONS.CLOSEST_SCORE_TABLE_GAME_PIECE_RIGHT, new PathPoint(new Translation2d(10.6, 0.9), new Rotation2d(180)));
    put(FIELD_POSITIONS.SECOND_CLOSEST_SCORE_TABLE_GAME_PIECE_RIGHT, new PathPoint(new Translation2d(10.6, 2.15), new Rotation2d(180)));
    put(FIELD_POSITIONS.THIRD_CLOSEST_SCORE_TABLE_GAME_PIECE_RIGHT, new PathPoint(new Translation2d(10.6, 3.37), new Rotation2d(180)));
    put(FIELD_POSITIONS.FOURTH_CLOSEST_SCORE_TABLE_GAME_PIECE_RIGHT, new PathPoint(new Translation2d(10.6, 4.59), new Rotation2d(180)));
    put(FIELD_POSITIONS.LEFT_SIDE_GRID_CONE_CLOSEST, new PathPoint(new Translation2d(2.16, 0.51), new Rotation2d(180)));
    put(FIELD_POSITIONS.LEFT_SIDE_GRID_CONE_SECOND_CLOSEST, new PathPoint(new Translation2d(2.16, 1.62), new Rotation2d(180)));
    put(FIELD_POSITIONS.LEFT_SIDE_GRID_CONE_THIRD_CLOSEST, new PathPoint(new Translation2d(2.16, 2.19), new Rotation2d(180)));
    put(FIELD_POSITIONS.LEFT_SIDE_GRID_CONE_FOURTH_CLOSEST, new PathPoint(new Translation2d(2.16, 3.30), new Rotation2d(180)));
    put(FIELD_POSITIONS.LEFT_SIDE_GRID_CONE_FIFTH_CLOSEST, new PathPoint(new Translation2d(2.16, 3.87), new Rotation2d(180)));
    put(FIELD_POSITIONS.LEFT_SIDE_GRID_CONE_SIXTH_CLOSEST, new PathPoint(new Translation2d(2.16, 4.99), new Rotation2d(180)));
    put(FIELD_POSITIONS.LEFT_SIDE_GRID_CUBE_CLOSEST, new PathPoint(new Translation2d(2.16, 1.04), new Rotation2d(180)));
    put(FIELD_POSITIONS.LEFT_SIDE_GRID_CUBE_SECOND_CLOSEST, new PathPoint(new Translation2d(2.16, 2.76), new Rotation2d(180)));
    put(FIELD_POSITIONS.LEFT_SIDE_GRID_CUBE_THIRD_CLOSEST, new PathPoint(new Translation2d(2.16, 4.43), new Rotation2d(180)));
    put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CONE_CLOSEST, new PathPoint(new Translation2d(14.4, 0.5), new Rotation2d(0)));
    put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CONE_SECOND_CLOSEST, new PathPoint(new Translation2d(14.4, 1.61), new Rotation2d(0)));
    put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CONE_THIRD_CLOSEST, new PathPoint(new Translation2d(14.4, 2.19), new Rotation2d(0)));
    put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CONE_FOURTH_CLOSEST, new PathPoint(new Translation2d(14.4, 3.29), new Rotation2d(0)));
    put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CONE_FIFTH_CLOSEST, new PathPoint(new Translation2d(14.4, 3.86), new Rotation2d(0)));
    put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CONE_SIXTH_CLOSEST, new PathPoint(new Translation2d(14.4, 4.98), new Rotation2d(0)));
    put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CUBE_CLOSEST, new PathPoint(new Translation2d(14.4, 0.49), new Rotation2d(0)));
    put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CUBE_SECOND_CLOSEST, new PathPoint(new Translation2d(14.4, 2.75), new Rotation2d(0)));
    put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CUBE_THIRD_CLOSEST, new PathPoint(new Translation2d(14.4, 4.41), new Rotation2d(0)));
    put(FIELD_POSITIONS.LEFT_SIDE_INTERMEDIATE_AUTO_CLOSEST, new PathPoint(new Translation2d(2.2, 0.6), new Rotation2d(0)));
    put(FIELD_POSITIONS.LEFT_SIDE_INTERMEDIATE_AUTO_SECOND_CLOSEST, new PathPoint(new Translation2d(2.2, 2.6), new Rotation2d(0)));
    put(FIELD_POSITIONS.LEFT_SIDE_INTERMEDIATE_AUTO_THIRD_CLOSEST, new PathPoint(new Translation2d(2.2, 4.6), new Rotation2d(0)));
    put(FIELD_POSITIONS.RIGHT_SIDE_INTERMEDIATE_AUTO_CLOSEST, new PathPoint(new Translation2d(14.4, 0.65), new Rotation2d(0)));
    put(FIELD_POSITIONS.RIGHT_SIDE_INTERMEDIATE_AUTO_SECOND_CLOSEST, new PathPoint(new Translation2d(14.4, 2.65), new Rotation2d(0)));
    put(FIELD_POSITIONS.RIGHT_SIDE_INTERMEDIATE_AUTO_THIRD_CLOSEST, new PathPoint(new Translation2d(14.4, 4.65), new Rotation2d(0)));
    }};


    private static HashMap<String, Command> eventMap = new HashMap<String, Command>()
    {{
         //put("Balance", new CmdAutoBalance());
    }};

    // Constants used in autonomous.
    private static final double MAX_VELOCITY = 3;
    private static final double MAX_ACCELERATION = 1.5;


  

    
    public static Command getMovementCommand(FIELD_POSITIONS targetPosition, Drive driveSubsystem, Consumer<ChassisSpeeds> chassisSpeed){
        // Create the trajectory to be executed to move to that position. 
        PathPlannerTrajectory targetTrajectory = new TrajectoryBuilder()
                                                .setStartPositionAsCurrent(driveSubsystem.getPose())
                                                .addConstraints(MAX_VELOCITY, MAX_ACCELERATION)
                                                .addPathPoint(fieldPositionsCoordinateMap.get(targetPosition))
                                                .buildTrajectory();

        
        /* The aformentioned builder takes the current postion of the robot and makes it the start of the trajectory. Then adds default limitations on the path
           and sets the field position which you are moving to as the end postion of the path, then finally builds it. */

        // Returns the final build trajectory to be executed. 
        return new PPSwerveControllerCommand(
            targetTrajectory,
            driveSubsystem::getPose,
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            chassisSpeed,
            driveSubsystem
        );
    }

    public static Command getPreloadedPathCommand(String pathName, double maxVel, double maxAccel, Drive driveSubsystem, Consumer<ChassisSpeeds> chassisSpeed){
        PathPlannerTrajectory path = PathPlanner.loadPath(pathName, maxVel, maxAccel);
        
        return new FollowPathWithEvents((new PPSwerveControllerCommand(
            path,
            driveSubsystem::getPose,
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            chassisSpeed,
            driveSubsystem
        )),path.getMarkers(), eventMap);
    }

    public static Command generateFullAuto(String autoName, HashMap<String, Command> eventMap, double maxVel, double maxAccel, Drive driveSubsystem) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(autoName, new PathConstraints(maxVel, maxAccel), new PathConstraints(maxVel, maxAccel));

        SwerveAutoBuilder builder = new SwerveAutoBuilder(driveSubsystem::getPose, 
        driveSubsystem::resetPose, 
            new PIDConstants(0, 0, 0),
            new PIDConstants(0, 0, 0), 
            driveSubsystem::drive, eventMap, false, driveSubsystem);

        return builder.fullAuto(pathGroup);
    }
}
