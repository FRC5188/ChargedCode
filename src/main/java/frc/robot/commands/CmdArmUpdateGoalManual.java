package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Arm2DPosition;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Arm.WristPosition;

public class CmdArmUpdateGoalManual extends CommandBase {
    private Arm _armSubsystem;
    private Arm2DPosition _position;

    public CmdArmUpdateGoalManual(Arm armSubsystem, Arm2DPosition position) {
        _armSubsystem = armSubsystem;
        _position = position;
        this.addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        _armSubsystem.setArmGoalsFromPosition(_position);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            System.out.println("Interrupted");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
