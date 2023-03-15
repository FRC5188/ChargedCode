package frc.robot.autonomous;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.GrpMoveArmAuto;
import frc.robot.subsystems.Arm;

public class GrpDriveScoreDriveBack {

    private PathPlannerTrajectory path;
    private HashMap<String, Command> eventMap;
    private Arm armSubsystem;


    public GrpDriveScoreDriveBack(Arm subsystem, String pathname){
        this.path = PathPlanner.loadPath(pathname, new PathConstraints(4, 3));
        this.eventMap = new HashMap<>();
        this.armSubsystem = subsystem;

        eventMap.put("marker1", new GrpMoveArmAuto(this.armSubsystem, Arm.ArmPosition.HighCube));

    //     FollowPathWithEvents command = new FollowPathWithEvents(
    //     ,
    //     examplePath.getMarkers(),
    //     eventMap
    // );
    }




    
}
