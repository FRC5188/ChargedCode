package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmMode;

public class CmdArmSetMode extends CommandBase {
  private Arm _armSubsystem;
  private ArmMode _armMode;

  public CmdArmSetMode(Arm armSubsystem, ArmMode mode) {
    _armSubsystem = armSubsystem;
    _armMode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _armSubsystem.setArmMode(_armMode);
    System.out.println("Set arm mode to " + _armSubsystem.getArmMode());
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
    return false;
  }
}
