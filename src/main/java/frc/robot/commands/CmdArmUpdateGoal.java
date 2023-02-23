// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Arm.WristPosition;

public class CmdArmUpdateGoal extends CommandBase {
  /** Creates a new CmdArmUpdateGoal. */
  Arm _armSubsystem;
  ArmPosition _armPosition;

  public CmdArmUpdateGoal(Arm armSubsystem, ArmPosition armPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    _armSubsystem = armSubsystem;
    _armPosition = armPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _armSubsystem.SetArmGoal(_armPosition);
    _armSubsystem.setWristPosition(_armPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
