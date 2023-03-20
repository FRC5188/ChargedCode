package frc.robot.arm.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;

public class CmdArmMoveElbowManual extends CommandBase {
    private Arm _armSubsystem;
    private DoubleSupplier _changeAmount;

    public CmdArmMoveElbowManual(Arm armSubsystem, DoubleSupplier changeAmount) {
        _changeAmount = changeAmount;
        _armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        _armSubsystem.setElbowGoalFromAngle(_armSubsystem.getElbowSetpoint() + _changeAmount.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
