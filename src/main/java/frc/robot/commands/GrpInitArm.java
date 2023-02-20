package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class GrpInitArm extends SequentialCommandGroup {

  public GrpInitArm(Arm armSubsystem) {
    addCommands(
      new CmdArmUpdateGoal(armSubsystem, ArmPosition.Stored),
      new CmdArmUpdateElbowGoal(armSubsystem),
      new CmdArmUpdateShoulderGoal(armSubsystem),
      new CmdArmWristPosition(armSubsystem)
    );
  }
}
