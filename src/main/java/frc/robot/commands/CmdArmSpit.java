
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class CmdArmSpit extends CommandBase {
    private Arm _armSubsystem;
    private double _intakeSpeed;
    private int _counter;

    public CmdArmSpit(Arm armSubsystem, double intakeSpeed) {
        this._armSubsystem = armSubsystem;
        this._intakeSpeed = intakeSpeed;
        this._counter = 0;

        this.addRequirements(_armSubsystem);
    }

    @Override
    public void initialize() {
        this._counter = 0;
    }

    @Override
    public void execute() {
        this._counter++;
        this._armSubsystem.setIntakeMotorSpeed(this._intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        _armSubsystem.setIntakeMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return this._counter >= 25;
    }
}
