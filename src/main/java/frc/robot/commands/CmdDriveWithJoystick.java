package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class CmdDriveWithJoystick extends CommandBase {
  private Drive _driveSubsystem;
  private DoubleSupplier _drive;
  private DoubleSupplier _strafe;
  private DoubleSupplier _rotate;

  public CmdDriveWithJoystick(Drive driveSubsystem, DoubleSupplier drive, DoubleSupplier strafe, DoubleSupplier rotate) {
    _driveSubsystem = driveSubsystem;
    _drive = drive;
    _strafe = strafe;
    _rotate = rotate;

    addRequirements(_driveSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    _driveSubsystem.drive(_drive.getAsDouble(), _strafe.getAsDouble(), _rotate.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
