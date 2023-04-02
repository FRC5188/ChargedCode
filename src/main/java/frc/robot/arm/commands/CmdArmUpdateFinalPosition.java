

package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;

public class CmdArmUpdateFinalPosition extends CommandBase {
  private Arm _armSubsystem;
  private ArmPosition _position;
  public CmdArmUpdateFinalPosition(Arm armSubsystem, ArmPosition position) {
    this._armSubsystem = armSubsystem;
    this._position = position;
    addRequirements(_armSubsystem);
  }

  @Override
  public void initialize() {
    _armSubsystem.setFinalPosition(_position);
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
