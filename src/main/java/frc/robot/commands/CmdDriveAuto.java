package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class CmdDriveAuto extends CommandBase {
    private Drive _driveSubsystem;
    private double _drive;
    private double _strafe;
    private double _rotate;

    public CmdDriveAuto(Drive driveSubsystem, double drive, double strafe, double rotate) {
        _driveSubsystem = driveSubsystem;
        _drive = drive;
        _strafe = strafe;
        _rotate = rotate;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        _driveSubsystem.drive(_drive, _strafe, _rotate);
    }

    @Override
    public void end(boolean interrupted) {
        _driveSubsystem.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
