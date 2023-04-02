

package frc.robot.arm.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.arm.commands.CmdArmUpdateFinalPosition;

public class GrpEngardeForScoring extends SequentialCommandGroup {
  public GrpEngardeForScoring(Arm armSubsystem, ArmPosition position) {
    
    addCommands(
      new CmdArmUpdateFinalPosition(armSubsystem, position),
      new GrpMoveArmToPosition(armSubsystem, ArmPosition.EnGarde)
    );
  }
}
