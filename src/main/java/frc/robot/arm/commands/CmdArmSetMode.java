package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmMode;

public class CmdArmSetMode extends CommandBase {
    private Arm _armSubsystem;
    private ArmMode _armMode;

    public CmdArmSetMode(Arm armSubsystem, ArmMode mode) {
        _armSubsystem = armSubsystem;
        _armMode = mode;
    }

    @Override
    public void initialize() {
        _armSubsystem.setArmMode(_armMode);
        System.out.println("Set arm mode to " + _armSubsystem.getArmMode());
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
