package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class CmdFollowSwerveAuto extends CommandBase {
    private final Drive _driveSubsystem;
    private final PPSwerveControllerCommand swerveControllerCommand;

    public CmdFollowSwerveAuto(Drive driveSubsystem, boolean isFirstPath, PathPlannerTrajectory trajectory){
        _driveSubsystem = driveSubsystem;
        this.addRequirements(_driveSubsystem);

        if(isFirstPath){_driveSubsystem.setGyroPose(trajectory.getInitialHolonomicPose());};

        // DOES NOT WORK!!!!
        swerveControllerCommand = new PPSwerveControllerCommand(
            trajectory, 
            _driveSubsystem.getPose(), 
            new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            _driveSubsystem.getRobotState(), 
            true,
            _driveSubsystem
            )
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
