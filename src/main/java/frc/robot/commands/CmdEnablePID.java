package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CmdEnablePID extends CommandBase{
    @Override
    public void initialize() {
        CmdArmDefault.enablePID();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
