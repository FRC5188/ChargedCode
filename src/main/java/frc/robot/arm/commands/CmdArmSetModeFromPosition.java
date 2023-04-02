package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmMode;
import frc.robot.arm.Arm.ArmPosition;

public class CmdArmSetModeFromPosition extends CommandBase {
  private Arm _armSubsystem;
  private ArmPosition _position;

  public CmdArmSetModeFromPosition(Arm armSubsystem, ArmPosition position) {
    _armSubsystem = armSubsystem;
    _position = position;
    addRequirements(_armSubsystem);
  }

  @Override
  public void initialize() {
    if (_position == ArmPosition.HighCone || _position == ArmPosition.MiddleCone) {
      _armSubsystem.setArmMode(ArmMode.Cone);
      System.out.println("CONE");
    } else {
      _armSubsystem.setArmMode(ArmMode.Cube);
      System.out.println("CUBE");
    }
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
