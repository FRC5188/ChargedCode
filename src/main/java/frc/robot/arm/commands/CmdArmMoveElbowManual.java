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
        double setpoint = _armSubsystem.getElbowSetpoint() + _changeAmount.getAsDouble();
        System.out.println("Changing Elbow from " + _armSubsystem.getElbowJointAngleRelativeToGround() + " to " + setpoint);
        _armSubsystem.setElbowGoalFromAngle(setpoint);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
