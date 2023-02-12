package frc.robot.subsystem_builders.Drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Drive;

public abstract class Autonomous{
    public static Command getTrajectoryCommand(PathPlannerTrajectory plannedTrajectory, boolean isFirstPath, Drive driveSubsystem){
            return new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        // Reset odometry for the first path you run during auto
                        if (isFirstPath) {
                            Odometry.resetPosition(
                                    Gyroscope.getGyroscopeRotation(),
                                    new SwerveModulePosition[]{
                                        Drivetrain.getFrontLeftModule().getModulePosition(), Drivetrain.getFrontLeftModule().getModulePosition(),
                                        Drivetrain.getBackLeftModule().getModulePosition(), Drivetrain.getBackRightModule().getModulePosition()},
                                    plannedTrajectory.getInitialHolonomicPose());
                        }
                    }),
    
                    new PPSwerveControllerCommand(plannedTrajectory,
                            Odometry::getPose, new PIDController(0, 0, 0),
                            new PIDController(0, 0, 0), new PIDController(0, 0, 0),
                            Drivetrain::drive, driveSubsystem));
    }
}