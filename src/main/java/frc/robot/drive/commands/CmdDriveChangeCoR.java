package frc.robot.drive.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.Drive;

public class CmdDriveChangeCoR extends CommandBase {
  private Drive _driveSubsystem;
  private Translation2d _cor;

  public CmdDriveChangeCoR(Drive driveSubsystem, Translation2d cor) {
    _driveSubsystem = driveSubsystem;
    _cor = cor;
  }

  @Override
  public void initialize() {
    _driveSubsystem.setCenterOfRotation(_cor);
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
