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
  
  public CmdArmWristPosition(Arm armSubsystem) {
    this._armSubsystem = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(_armSubsystem); 
  }

  @Override
    public void initialize() {
      ArmPosition position = _armSubsystem.getCurrentArmPosition();
      _armSubsystem.setWristPosition(position);
      System.out.println("set wrist position to " + position);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
