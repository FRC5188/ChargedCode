package frc.robot.arm.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;

public class GrpAutoEngardeScoreStow extends SequentialCommandGroup {
  public GrpAutoEngardeScoreStow(Arm armSubsystem, ArmPosition position) {
    addCommands(
      new GrpAutoGoToScore(armSubsystem, position),
      new GrpScoreAndStow(armSubsystem)
    );
  }
}
