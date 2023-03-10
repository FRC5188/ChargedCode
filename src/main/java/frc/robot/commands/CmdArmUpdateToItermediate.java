package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class CmdArmUpdateToItermediate extends CommandBase {
  private Arm _armSubsystem;
  private ArmPosition _position;

  public CmdArmUpdateToItermediate(Arm armSubsystem, ArmPosition position) {
    _armSubsystem = armSubsystem;
    _position = position;

    this.addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    ArmPosition pos = _armSubsystem.getIntermediatePosition(_position);
    _armSubsystem.setArmGoalsFromPosition(pos);
    _armSubsystem.setWristPosition(pos);
    System.out.println("Updated arm position to " + pos);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
