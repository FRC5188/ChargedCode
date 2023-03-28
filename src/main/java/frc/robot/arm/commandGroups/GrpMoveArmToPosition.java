package frc.robot.arm.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.arm.commands.CmdArmRunIntake;
import frc.robot.arm.commands.CmdArmUpdateGoal;
import frc.robot.arm.commands.CmdArmUpdateToItermediate;
import frc.robot.arm.commands.CmdArmWaitForArm;
import frc.robot.LEDs.commands.CmdLEDPieceCollected;
import frc.robot.LEDs.*;

public class GrpMoveArmToPosition extends SequentialCommandGroup {

  //

  public GrpMoveArmToPosition(Arm armSubsystem, ArmPosition position) {
    this.addRequirements(armSubsystem);

    //

    addCommands(
      
        // Now go to actual position
        new CmdArmUpdateGoal(armSubsystem, position),
        // Turn on the intake if needed

        new CmdArmRunIntake(armSubsystem, 0.4)
            .unless(() -> position != ArmPosition.GroundPickUp &&
              position != ArmPosition.LoadStationPickUp),
        // Added for auto. Will keep running until arm is in position
        new CmdArmWaitForArm(armSubsystem)

    );
  }
}
