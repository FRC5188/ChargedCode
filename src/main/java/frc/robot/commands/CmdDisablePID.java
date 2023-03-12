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
        _armsubsystem.setShoulderMotorSpeed(0);
        _armsubsystem.setElbowMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
