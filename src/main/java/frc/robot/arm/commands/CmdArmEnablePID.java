package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;

public class CmdArmEnablePID extends CommandBase {
    private Arm _armSubsystem;

    public CmdArmEnablePID(Arm armSubsystem) {
        _armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {
        _armSubsystem.enablePID();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
