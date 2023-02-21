package frc.robot.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
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

    public static void movePosition(FIELD_POSITIONS targetPosition, Drive driveSubsystem){
        // Create the trajectory to be executed to move to that position. 
        PathPlannerTrajectory targetTrajectory = new TrajectoryBuilder()
                                                .setStartPositionAsCurrent(driveSubsystem.getPose())
                                                .addConstraints(3, 4)
                                                // Doesn't Work Add Points For Each Field Position
                                                .buildTrajectory();
        // Execute trajectory...
    }
}
