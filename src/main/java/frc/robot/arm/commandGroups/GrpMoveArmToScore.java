

package frc.robot.arm.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.CmdArmSpit;
import frc.robot.arm.commands.CmdArmUpdateToScorePos;
import frc.robot.arm.commands.CmdArmWaitForArm;


public class GrpMoveArmToScore extends SequentialCommandGroup {

  public GrpMoveArmToScore(Arm armSubsystem) {

    addCommands(
     new CmdArmUpdateToScorePos(armSubsystem),
     new CmdArmWaitForArm(armSubsystem),
     new CmdArmSpit(armSubsystem, 0.4)
    );
  }
}
