package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;

public class CmdArmUpdateGoal extends CommandBase {
    private Arm _armSubsystem;
    private ArmPosition _position;

    public CmdArmUpdateGoal(Arm armSubsystem, ArmPosition position) {
        _armSubsystem = armSubsystem;
        _position = position;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        _armSubsystem.setWristPosition(_position);
        _armSubsystem.generateTrajectory(_position);
        _armSubsystem.startTrajectory();
        _armSubsystem.setCanChangeSetpoint(false);
        
        System.out.println("Updated arm position to " + _position);
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
