

package frc.robot.arm.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.arm.commands.CmdArmSpit;
import frc.robot.arm.commands.CmdArmUpdateFinalPosition;
import frc.robot.arm.commands.CmdArmWaitForArm;
import frc.robot.arm.commands.CmdWait;

public class GrpScoreAndStow extends SequentialCommandGroup {
  public GrpScoreAndStow(Arm armSubsystem) {
    this.addRequirements(armSubsystem);

    addCommands(
      new CmdArmSpit(armSubsystem, -0.6),
      new GrpMoveArmToPosition(armSubsystem, ArmPosition.EnGarde),
      new GrpMoveArmToPosition(armSubsystem, ArmPosition.Stored),
      new CmdArmUpdateFinalPosition(armSubsystem, ArmPosition.Stored)
    );
  }
}
