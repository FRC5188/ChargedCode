package frc.robot.arm.commandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmMode;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.arm.commands.CmdArmSetModeFromPosition;
import frc.robot.arm.commands.CmdArmUpdateToFinalPosition;
import frc.robot.arm.commands.CmdArmWaitForArm;

public class GrpEngardeScoreAndStow extends SequentialCommandGroup {
  public GrpEngardeScoreAndStow(Arm armSubsystem, ArmPosition position) {
    addCommands(
      new InstantCommand(() -> System.out.println("RUNNING AUTO")),
      new CmdArmSetModeFromPosition(armSubsystem, position),
      new GrpEngardeForScoring(armSubsystem, position),
      new CmdArmUpdateToFinalPosition(armSubsystem),
      new CmdArmWaitForArm(armSubsystem),
      new GrpScoreAndStow(armSubsystem)
    );
  }
}
