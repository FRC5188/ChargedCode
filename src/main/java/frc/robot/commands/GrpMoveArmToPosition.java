// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

/** Sequential Command Group which moves the arm to the target passed in. */
public class GrpMoveArmToPosition extends SequentialCommandGroup {
  private Arm _armSubsystem;
  private ArmPosition _targetPosition;

  public GrpMoveArmToPosition(Arm armSubsystem, ArmPosition targetPosition){
    this._armSubsystem = armSubsystem;
    this._targetPosition = targetPosition;

    addRequirements(this._armSubsystem);

    addCommands(
      
    );
  }
}
