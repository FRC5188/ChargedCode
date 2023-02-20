package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class CmdArmUpdateShoulderGoal extends CommandBase {
  private Arm _armSubsystem;
  private ArmPosition _position;

  public CmdArmUpdateShoulderGoal(Arm armSubsystem) {
    _armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    _position = _armSubsystem.getCurrentArmPosition();
    _armSubsystem.shoulderPIDSetGoal(_position);
    System.out.println("Updated Shoulder Goal to " + _position);
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
