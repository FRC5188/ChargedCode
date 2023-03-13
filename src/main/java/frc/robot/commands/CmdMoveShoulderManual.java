package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class CmdMoveShoulderManual extends CommandBase {

    private final double changeAmount;
    private Arm _armSubsystem;

    /*
     * @param changeAmount, is in degrees
     */
    public CmdMoveShoulderManual(Arm armSubsystem, double changeAmount) {
        this.addRequirements(armSubsystem);
        this.changeAmount = changeAmount;
        this._armSubsystem = armSubsystem;

    }

    @Override
    public void initialize() {
        double setPoint = _armSubsystem.getShoulderSetpoint() + changeAmount;

        _armSubsystem.setShoulderGoalFromAngle(setPoint);
    }
    
    @Override
    public boolean isFinished() {
      return true;
    }

}
