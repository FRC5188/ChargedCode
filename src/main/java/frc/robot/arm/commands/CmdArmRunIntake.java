package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmMode;
import frc.robot.arm.Arm.IntakeMode;

public class CmdArmRunIntake extends CommandBase {
    private Arm _armSubsystem;
    private double _intakeSpeed;

    public CmdArmRunIntake(Arm armSubsystem, double intakeSpeed) {
        this._armSubsystem = armSubsystem;
        this._intakeSpeed = intakeSpeed;

        this.addRequirements(_armSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        // If the gampiece is a cone, close the intake
        if (_armSubsystem.getArmMode() == ArmMode.Cone) {
            _armSubsystem.setIntakeMode(IntakeMode.Closed);
        } else {
            // Keep it open if it is a cube
            _armSubsystem.setIntakeMode(IntakeMode.Open);
        }

        _armSubsystem.setIntakeMotorSpeed(this._intakeSpeed);

    }

    @Override
    public void end(boolean interrupted) {
        // Once the intake has a piece, stop the motors and close the intake
        _armSubsystem.setIntakeMotorSpeed(0);
        _armSubsystem.setIntakeMode(IntakeMode.Closed);
    }

    @Override
    public boolean isFinished() {
        return _armSubsystem.intakeHasPiece();
    }
}
