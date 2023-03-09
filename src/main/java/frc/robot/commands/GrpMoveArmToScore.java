

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;


public class GrpMoveArmToScore extends SequentialCommandGroup {

  public GrpMoveArmToScore(Arm armSubsystem, ArmPosition armPosition) {

    addCommands(
     new CmdArmUpdateGoal(armSubsystem, armPosition),
     new CmdArmWaitForArm(armSubsystem),
     new CmdArmRunIntake(armSubsystem, 0.4)
    );
  }
}
