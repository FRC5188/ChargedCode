package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;

public class CmdArmUpdateToLastTarget extends CommandBase {
    private Arm _armSubsystem;

    public CmdArmUpdateToLastTarget(Arm armSubsystem) {
        _armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        ArmPosition pos = _armSubsystem.getTargetArmPosition();
        _armSubsystem.setWristPosition(pos);
        _armSubsystem.generateTrajectory(pos);
        _armSubsystem.startTrajectory();
        _armSubsystem.setCanChangeSetpoint(false);
        
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
