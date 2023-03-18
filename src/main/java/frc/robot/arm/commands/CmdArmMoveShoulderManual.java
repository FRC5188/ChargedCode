package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;

public class CmdArmMoveShoulderManual extends CommandBase {
    private Arm _armSubsystem;
    private double _changeAmount;

    public CmdArmMoveShoulderManual(Arm armSubsystem, double changeAmount) {
        _armSubsystem = armSubsystem;
        _changeAmount = changeAmount;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        _armSubsystem.setShoulderGoalFromAngle(_armSubsystem.getShoulderSetpoint() + _changeAmount);
    }
    
    @Override
    public boolean isFinished() {
      return true;
    }
}
