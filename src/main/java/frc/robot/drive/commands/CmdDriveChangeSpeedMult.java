// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.Drive;

public class CmdDriveChangeSpeedMult extends CommandBase {
  private double _speedMultiplier;
  private Drive _driveSubsystem;
  /** Creates a new CmdDriveChangeSpeedMult. */
  public CmdDriveChangeSpeedMult(Drive driveSubsystem, double multiplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    this._driveSubsystem = driveSubsystem;
    this._speedMultiplier = multiplier;
    addRequirements(_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this._driveSubsystem.setSpeedMultiplier(this._speedMultiplier);
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
