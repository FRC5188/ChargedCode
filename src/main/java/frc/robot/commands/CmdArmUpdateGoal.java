package frc.robot.commands;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class CmdArmUpdateGoal extends CommandBase {
    private Arm _armSubsystem;
    private ArmPosition _position;

    public CmdArmUpdateGoal(Arm armSubsystem, ArmPosition position) {
        _armSubsystem = armSubsystem;
        _position = position;
    }

    @Override
    public void initialize() {
        _armSubsystem.setCurrentArmPosition(_position);
        // _armSubsystem.shoulderPIDSetGoal(_position);
        // _armSubsystem.elbowPIDSetGoal(_position);
        _armSubsystem.setDesiredWristPosition(_position);
        System.out.println("Updated arm position to " + _position);
        System.out.println("Current position is " + _armSubsystem.getCurrentArmPosition());
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
