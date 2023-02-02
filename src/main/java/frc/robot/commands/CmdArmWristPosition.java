// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Arm.WristPosition;

public class CmdArmWristPosition extends CommandBase {
  /** Creates a new CmdArmWristPosition. */
  private Arm _armSubsystem;
  private ArmPosition _setpoint;
  
  public CmdArmWristPosition(Arm armSubsystem, ArmPosition setpoint) {
    this._armSubsystem = armSubsystem;
    this._setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(_armSubsystem); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _armSubsystem.setWristPosition(_setpoint);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
