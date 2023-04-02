package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CmdWait extends CommandBase {
    private int _timeInMs;
    private int _counter;
    private int _countUntil;

    public CmdWait(int timeInMs) {
        _timeInMs = timeInMs;
    }

    @Override
    public void initialize() {
        _counter = 0;
        _countUntil = _timeInMs / 20;
    }

    @Override
    public void execute() {
        _counter++;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return _counter >= _countUntil;
    }
}
