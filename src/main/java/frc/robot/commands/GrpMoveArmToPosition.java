package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class GrpMoveArmToPosition extends SequentialCommandGroup {
  public GrpMoveArmToPosition(Arm armSubsystem, ArmPosition position) {

    addCommands(
      // Update arm goal to the intermediate position
      new CmdArmUpdateGoal(armSubsystem, armSubsystem.getIntermediatePosition(position)),
      // Wait for us to get there
      new CmdArmWaitForArm(armSubsystem),
      // Now go to actual position
      new CmdArmUpdateGoal(armSubsystem, position)
    );
  }
}
