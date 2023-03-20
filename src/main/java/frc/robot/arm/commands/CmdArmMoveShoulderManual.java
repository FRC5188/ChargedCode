package frc.robot.arm.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;

public class CmdArmMoveShoulderManual extends CommandBase {
    private Arm _armSubsystem;
    private DoubleSupplier _changeAmount;

    public CmdArmMoveShoulderManual(Arm armSubsystem, DoubleSupplier changeAmount) {
        _armSubsystem = armSubsystem;
        _changeAmount = changeAmount;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        _armSubsystem.setShoulderGoalFromAngle(_armSubsystem.getShoulderSetpoint() + _changeAmount.getAsDouble());
    }
    
    @Override
    public boolean isFinished() {
      return true;
    }
}
