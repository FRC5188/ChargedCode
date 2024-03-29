// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.Drive;

public class CmdDriveResetGyro extends CommandBase {
  /** Creates a new CmdDriveResetGyro. */
  private Drive _driveSubsystem;
  public CmdDriveResetGyro(Drive driveSubsystem) {
    this._driveSubsystem = driveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _driveSubsystem.zeroGyroscope();
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
