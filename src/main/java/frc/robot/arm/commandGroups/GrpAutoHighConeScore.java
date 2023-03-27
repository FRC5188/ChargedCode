// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.commandGroups;

import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.arm.Arm.WristPosition;
import frc.robot.arm.commands.CmdArmSetWristPosition;
import frc.robot.arm.commands.CmdArmSetWristPositionManual;
import frc.robot.arm.commands.CmdArmSpit;
import frc.robot.arm.commands.CmdArmUpdateToScorePos;
import frc.robot.arm.commands.CmdArmWaitForArm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrpAutoHighConeScore extends SequentialCommandGroup {
  /** Creates a new GrpAutoHighConeScore. */
  public GrpAutoHighConeScore(Arm armSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new GrpMoveArmToPosition(armSubsystem, ArmPosition.EnGarde), 
      new CmdArmWaitForArm(armSubsystem),
      new GrpMoveArmToPosition(armSubsystem, ArmPosition.HighCone), 
      new CmdArmUpdateToScorePos(armSubsystem),
      new CmdArmSetWristPositionManual(armSubsystem, WristPosition.Parallel),
      new CmdArmWaitForArm(armSubsystem),
      new CmdArmSpit(armSubsystem, 0.4),
      new GrpMoveArmToPosition(armSubsystem, ArmPosition.EnGarde)
    );
  }
}
