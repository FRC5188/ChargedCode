package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class CmdDisablePID extends CommandBase{

    private Arm _armsubsystem;

    public CmdDisablePID(Arm armsubsystem){
    this._armsubsystem = armsubsystem;
    }
    @Override
    public void initialize() {
        this._armsubsystem.disablePID();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
