// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class CmdArmSetElbowBrakeMode extends CommandBase {
  private Arm _armSubsystem;
  private NeutralMode _mode;
  /** Creates a new CmdArmSetElbowBrakeMode. */
  public CmdArmSetElbowBrakeMode(Arm armSubsystem, NeutralMode mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    _armSubsystem = armSubsystem;
    _mode = mode;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _armSubsystem.elbowMotorPIDInit(0);
    _armSubsystem.setElbowBrakeMode(_mode);
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
