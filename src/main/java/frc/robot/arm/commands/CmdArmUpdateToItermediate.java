package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;

public class CmdArmUpdateToItermediate extends CommandBase {
    private Arm _armSubsystem;
    private ArmPosition _position;

    public CmdArmUpdateToItermediate(Arm armSubsystem, ArmPosition position) {
        _armSubsystem = armSubsystem;
        _position = position;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        ArmPosition pos = _armSubsystem.getIntermediatePositions(_position).get(0);
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
