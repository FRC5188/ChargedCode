package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class CmdArmWaitForArm extends CommandBase {
  private Arm _armSubsystem;
  private boolean _shoulderDone = false;
  private boolean _elbowDone = false;
  
  public CmdArmWaitForArm(Arm armSubsystem) {
    _armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    System.out.println("Waiting for arm to reach setpoint...");
  }

  @Override
  public void execute() {
    // Once we say we are at setpoint once, we are close enough to move on (this is for intermediate positioning mostly)
    if (!_shoulderDone) {
      _shoulderDone = _armSubsystem.shoulderAtSetpoint();
    }

    if (!_elbowDone) {
      _elbowDone = _armSubsystem.elbowAtSetpoint();
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Arm at setpoint");
  }

  @Override
  public boolean isFinished() {
    return _shoulderDone && _elbowDone;
  }
}
