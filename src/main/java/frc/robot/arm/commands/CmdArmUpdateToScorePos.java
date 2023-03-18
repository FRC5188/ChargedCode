package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;

public class CmdArmUpdateToScorePos extends CommandBase {
    private Arm _armSubsystem;

    public CmdArmUpdateToScorePos(Arm armSubsystem) {
        _armSubsystem = armSubsystem;

        this.addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        ArmPosition pos = _armSubsystem.getScoringPosition(_armSubsystem.getCurrentArmPosition());
        _armSubsystem.setArmGoalsFromPosition(pos);
        _armSubsystem.setWristPosition(pos);
        
        System.out.println("Updated arm position to " + pos);
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
