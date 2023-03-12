package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CmdDisablePID extends CommandBase{
    @Override
    public void initialize() {
        CmdArmDefault.disablePID();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
