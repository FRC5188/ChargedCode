package frc.robot.test.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.arm.commandGroups.GrpMoveArmToPosition;
import frc.robot.test.console_output.Output;

@Deprecated
public class Arm {
    private static frc.robot.arm.Arm subsystem;
    /**
    The Arm will iterate through all position in a method which is meant to represent
    gameplay. For example, though low cone might only be run once, high cone and ground 
    pickup will be run multiple times. 
    **/
    public static Command runChecks(frc.robot.arm.Arm subsystem){
        Arm.subsystem = subsystem;

        // This will take a while just fyi.
        return new SequentialCommandGroup(
            // Low Goal Cycle (Ground Pickup)
            setArmPosition(ArmPosition.GroundPickUp),
            setArmPosition(ArmPosition.Stored),
            setArmPosition(ArmPosition.EnGarde),
            setArmPosition(ArmPosition.LowScore),
            setArmPosition(ArmPosition.EnGarde),
            setArmPosition(ArmPosition.Stored),
            new InstantCommand(
                () -> {Output.sucess("Low Goal Cycle Complete.");}
            ),


            // Middle Goal Cycle (Cone & Cube)
            setArmPosition(ArmPosition.GroundPickUp),
            setArmPosition(ArmPosition.Stored),
            setArmPosition(ArmPosition.EnGarde),
            setArmPosition(ArmPosition.MiddleCone),
            setArmPosition(ArmPosition.EnGarde),
            setArmPosition(ArmPosition.Stored),
            // That was for cone. This one is cone but for human player.

            setArmPosition(ArmPosition.EnGarde),
            setArmPosition(ArmPosition.LoadStationPickUp),
            setArmPosition(ArmPosition.EnGarde),
            setArmPosition(ArmPosition.Stored),
            setArmPosition(ArmPosition.EnGarde),
            setArmPosition(ArmPosition.MiddleCube),
            setArmPosition(ArmPosition.EnGarde),
            setArmPosition(ArmPosition.Stored),
            new InstantCommand(
                () -> {Output.sucess("Middle Goal Cycle Complete.");}
            ),

            // High Goal Cycle (Cone & Cube)
            setArmPosition(ArmPosition.GroundPickUp),
            setArmPosition(ArmPosition.Stored),
            setArmPosition(ArmPosition.EnGarde),
            setArmPosition(ArmPosition.HighCone),
            setArmPosition(ArmPosition.EnGarde),
            setArmPosition(ArmPosition.Stored),
            // That was for cone. This one is cone but for human player.

            setArmPosition(ArmPosition.EnGarde),
            setArmPosition(ArmPosition.LoadStationPickUp),
            setArmPosition(ArmPosition.EnGarde),
            setArmPosition(ArmPosition.Stored),
            setArmPosition(ArmPosition.EnGarde),
            setArmPosition(ArmPosition.HighCube),
            new InstantCommand(
                () -> {Output.sucess("High Goal Cycle Complete.");}
            ),


            // Back to store. 
            setArmPosition(ArmPosition.EnGarde),
            setArmPosition(ArmPosition.Stored),
            new InstantCommand(
                () -> {Output.sucess("All Cycles Complete.");}
            )
        );
    }

    private static Command setArmPosition(ArmPosition armPosition){
        // Since there isn't a singleton implementation of the Arm subsystem we have to pass it in. 
        return new GrpMoveArmToPosition(Arm.subsystem, armPosition);
    }
}
