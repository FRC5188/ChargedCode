package frc.robot.autonomous;

import java.util.HashMap;
import java.util.function.Consumer;

import javax.print.attribute.standard.Fidelity;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
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
    }
    /** Assocaites each positon that robot could be at with a PathPoint used to create on-the-fly autonomous. */
    private static HashMap<FIELD_POSITIONS, PathPoint> fieldPositionsCoordinateMap = new HashMap<>()
    // Creates the map for the field positions. 
    {
        {
            // TODO: Add Coordiantes from FRC PathPlanner 
            put(FIELD_POSITIONS.SINGLE_SUBSTRATION_LEFT, new PathPoint(null, null));
            put(FIELD_POSITIONS.SINGLE_SUBSTRATION_RIGHT, new PathPoint(null, null));
            put(FIELD_POSITIONS.DOUBLE_SUBSTRATION_LEFT, new PathPoint(null, null));
            put(FIELD_POSITIONS.DOUBLE_SUBSTRATION_RIGHT, new PathPoint(null, null));
            put(FIELD_POSITIONS.CHARGE_STATION_LEFT, new PathPoint(null, null));
            put(FIELD_POSITIONS.CHARGE_STATION_RIGHT, new PathPoint(null, null));
            put(FIELD_POSITIONS.CLOSEST_SCORE_TABLE_GAME_PIECE_LEFT, new PathPoint(null, null));
            put(FIELD_POSITIONS.SECOND_CLOSEST_SCORE_TABLE_GAME_PIECE_LEFT, new PathPoint(null, null));
            put(FIELD_POSITIONS.THIRD_CLOSEST_SCORE_TABLE_GAME_PIECE_LEFT, new PathPoint(null, null));
            put(FIELD_POSITIONS.FOURTH_CLOSEST_SCORE_TABLE_GAME_PIECE_LEFT, new PathPoint(null, null));
            put(FIELD_POSITIONS.CLOSEST_SCORE_TABLE_GAME_PIECE_RIGHT, new PathPoint(null, null));
            put(FIELD_POSITIONS.SECOND_CLOSEST_SCORE_TABLE_GAME_PIECE_RIGHT, new PathPoint(null, null));
            put(FIELD_POSITIONS.THIRD_CLOSEST_SCORE_TABLE_GAME_PIECE_RIGHT, new PathPoint(null, null));
            put(FIELD_POSITIONS.LEFT_SIDE_GRID_CONE_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.LEFT_SIDE_GRID_CONE_SECOND_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.LEFT_SIDE_GRID_CONE_THIRD_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.LEFT_SIDE_GRID_CONE_THIRD_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.LEFT_SIDE_GRID_CONE_FOURTH_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.LEFT_SIDE_GRID_CONE_FIFTH_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.LEFT_SIDE_GRID_CONE_SIXTH_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.LEFT_SIDE_GRID_CUBE_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.LEFT_SIDE_GRID_CUBE_SECOND_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.LEFT_SIDE_GRID_CUBE_THIRD_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CONE_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CONE_SECOND_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CONE_THIRD_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CONE_FOURTH_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CONE_FIFTH_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CONE_SIXTH_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CUBE_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CUBE_SECOND_CLOSEST, new PathPoint(null, null));
            put(FIELD_POSITIONS.RIGHT_SIDE_GRID_CUBE_THIRD_CLOSEST, new PathPoint(null, null));
        }
    };

    public static Command getMovementCommand(FIELD_POSITIONS targetPosition, Drive driveSubsystem, Consumer<ChassisSpeeds> chassisSpeed){
        // Create the trajectory to be executed to move to that position. 
        PathPlannerTrajectory targetTrajectory = new TrajectoryBuilder()
                                                .setStartPositionAsCurrent(driveSubsystem.getPose())
                                                .addConstraints(3, 4)
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

}
