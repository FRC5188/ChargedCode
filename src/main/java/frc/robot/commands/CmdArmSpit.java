
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Arm.IntakeMode;

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

        /**
         * IF the arm is holding a cone, it only opens the claw without running the wheels.
         */
        if (_armSubsystem.getArmMode() == ArmMode.Cone) {
            _armSubsystem.setIntakeMode(IntakeMode.OPEN);
        }

        /**
         * If the arm is holding a cube, it just spits out the cube by runnning the intake wheels.
         */
        if (_armSubsystem.getArmMode() == ArmMode.Cube) {
            this._counter++;
            this._armSubsystem.setIntakeMotorSpeed(this._intakeSpeed);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        _armSubsystem.setIntakeMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {

        if (_armSubsystem.getArmMode() == ArmMode.Cone) {
            return true;
        } else {
            return this._counter >= 25;
        }
    }
}
