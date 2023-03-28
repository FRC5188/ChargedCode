
package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;

public class CmdArmUpdateToFinalPosition extends CommandBase {
  private Arm _armSubsystem;

  public CmdArmUpdateToFinalPosition(Arm armSubsystem) {
    this._armSubsystem = armSubsystem;

    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    ArmPosition pos = _armSubsystem.getFinalPosition();
    _armSubsystem.setWristPosition(pos);
    _armSubsystem.generateTrajectory(pos);
    _armSubsystem.startTrajectory();

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
