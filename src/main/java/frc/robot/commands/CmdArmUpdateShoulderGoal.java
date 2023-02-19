package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class CmdArmUpdateShoulderGoal extends CommandBase {
  private Arm _armSubsystem;
  private ArmPosition _position;

  public CmdArmUpdateShoulderGoal(Arm armSubsystem, ArmPosition position) {
    _armSubsystem = armSubsystem;
    _position = position;
  }

  @Override
  public void initialize() {
    _armSubsystem.shoulderPIDSetGoal(_position);
    System.out.println("Updated Elbow Goal");
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
