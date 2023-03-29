// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.commandGroups;

import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.arm.Arm.WristPosition;
import frc.robot.arm.commands.CmdArmSetModeFromPosition;
import frc.robot.arm.commands.CmdArmSetWristPosition;
import frc.robot.arm.commands.CmdArmSetWristPositionManual;
import frc.robot.arm.commands.CmdArmSpit;
import frc.robot.arm.commands.CmdArmUpdateFinalPosition;
import frc.robot.arm.commands.CmdArmUpdateToFinalPosition;
import frc.robot.arm.commands.CmdArmUpdateToScorePos;
import frc.robot.arm.commands.CmdArmWaitForArm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GrpAutoGoToScore extends SequentialCommandGroup {
  public GrpAutoGoToScore(Arm armSubsystem, ArmPosition position) {
    addCommands(
      new CmdArmSetModeFromPosition(armSubsystem, position),
      new CmdArmUpdateFinalPosition(armSubsystem, ArmPosition.High),
      new GrpMoveArmToPosition(armSubsystem, ArmPosition.EnGarde), 
      new CmdArmWaitForArm(armSubsystem),
      new CmdArmUpdateToFinalPosition(armSubsystem),
      new CmdArmWaitForArm(armSubsystem)
    );
  }
}
