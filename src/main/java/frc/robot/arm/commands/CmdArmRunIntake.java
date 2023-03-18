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

        //If the gampiece is a cone, close the intake and run the motors.
         if (_armSubsystem.getArmMode() == ArmMode.Cone) {
            _armSubsystem.setIntakeMode(IntakeMode.Closed);
            this._armSubsystem.setIntakeMotorSpeed(this._intakeSpeed);
        }
        
        //If the gampiece is a cube, open the intake and run the motors.
        if (_armSubsystem.getArmMode() == ArmMode.Cube) {
            _armSubsystem.setIntakeMode(IntakeMode.Open);            
            this._armSubsystem.setIntakeMotorSpeed(this._intakeSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        _armSubsystem.setIntakeMotorSpeed(0);
        _armSubsystem.setIntakeMode(IntakeMode.Closed);
    }

    @Override
    public boolean isFinished() {
        return _armSubsystem.intakeHasPiece();
    }
}
