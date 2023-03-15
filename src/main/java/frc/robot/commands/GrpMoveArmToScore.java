

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;


public class GrpMoveArmToScore extends SequentialCommandGroup {

  public GrpMoveArmToScore(Arm armSubsystem) {

    addCommands(
     new CmdArmUpdateToScorePos(armSubsystem),
     new CmdArmWaitForArm(armSubsystem),
     new CmdArmSpit(armSubsystem, 0.4)
    );
  }
}