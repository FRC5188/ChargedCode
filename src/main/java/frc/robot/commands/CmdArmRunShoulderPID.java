// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class CmdArmRunShoulderPID extends CommandBase {

  private Arm _armSubsystem;
  private ArmPosition _setpoint;

  /** Creates a new CmdArmRunShoulderPID. */
  public CmdArmRunShoulderPID(Arm armSubsystem, ArmPosition setpoint) {
    this._armSubsystem = armSubsystem;
    this._setpoint = setpoint;

    this.addRequirements(_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _armSubsystem.shoulderMotorPIDInit(_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _armSubsystem.shoulderMotorPIDExec();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _armSubsystem.shoulderMotorPIDIsFinished();
  }
}
