package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class CmdArmUpdateElbowGoal extends CommandBase {
  private Arm _armSubsystem;
  private ArmPosition _position;

  public CmdArmUpdateElbowGoal(Arm armSubsystem) {
    _armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    _position = _armSubsystem.getCurrentArmPosition();
    _armSubsystem.elbowPIDSetGoal(_position);
    System.out.println("Updated Elbow Goal to " + _position);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
