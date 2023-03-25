package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmMode;
import frc.robot.arm.Arm.IntakeMode;

public class CmdArmRunIntake extends CommandBase {
    private Arm _armSubsystem;
    private double _intakeSpeed;
    private int _counter;

    public CmdArmRunIntake(Arm armSubsystem, double intakeSpeed) {
        this._armSubsystem = armSubsystem;
        this._intakeSpeed = intakeSpeed;
        _counter = 0;

        this.addRequirements(_armSubsystem);
    }

    @Override
    public void initialize() {
        _counter = 0;
    }

    @Override
    public void execute() {

        // If the gampiece is a cone, close the intake
        if (_armSubsystem.getArmMode() == ArmMode.Cone) {
            System.out.println("Cone Intake");
            _armSubsystem.setIntakeMode(IntakeMode.Closed);
        } else {
            // Keep it open if it is a cube
            System.out.println("Cube Intake");
            _armSubsystem.setIntakeMode(IntakeMode.Open);
        }

        _armSubsystem.setIntakeMotorSpeed(this._intakeSpeed);
        _counter++;
    }

    @Override
    public void end(boolean interrupted) {
        // Once the intake has a piece, stop the motors and close the intake
        System.out.println("Intake has piece");
        _armSubsystem.setIntakeMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return _armSubsystem.intakeHasPiece() && _counter > 10;
    }
}
