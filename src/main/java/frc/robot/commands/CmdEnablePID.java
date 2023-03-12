package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class CmdEnablePID extends CommandBase{
    private Arm _armSubsystem;
    public CmdEnablePID(Arm _armSubsystem) {
        this._armSubsystem = _armSubsystem;
    }

    @Override
    public void initialize() {
        _armSubsystem.EnablePID();;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
