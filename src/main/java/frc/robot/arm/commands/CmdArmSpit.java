package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmMode;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.arm.Arm.IntakeMode;

public class CmdArmSpit extends CommandBase {
    private Arm _armSubsystem;
    private double _intakeSpeed;
    private int _counter;

    public CmdArmSpit(Arm armSubsystem, double intakeSpeed) {
        _armSubsystem = armSubsystem;
        _intakeSpeed = intakeSpeed;
        _counter = 0;

        addRequirements(_armSubsystem);
    }

    @Override
    public void initialize() {
        _counter = 0;
    }

    @Override
    public void execute() {
        // If the arm is holding a cone, it only opens the claw without running the
        // wheels.
        // Unless we are in stowed, in which case we are spitting with wheels so we can
        // launch
        // out of the robot's perimeter
        if (_armSubsystem.getArmMode() == ArmMode.Cone && (_armSubsystem.getCurrentArmPosition() != ArmPosition.Stored
                && _armSubsystem.getCurrentArmPosition() != ArmPosition.LowScore)) {
            _armSubsystem.setIntakeMode(IntakeMode.Open);
            System.out.println("ACTUATE");
        } else {
            _armSubsystem.setIntakeMotorSpeed(_intakeSpeed);
            _counter++;
        }

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("INTAKE DONE");
        _armSubsystem.setIntakeMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        // If we are dropping a cone, stop immediately
        // If it is a cube, run the wheels for a bit
        return (_armSubsystem.getArmMode() == ArmMode.Cone
                && (_armSubsystem.getCurrentArmPosition() != ArmPosition.Stored
                        && _armSubsystem.getCurrentArmPosition() != ArmPosition.LowScore))
                || _counter >= 25;
    }
}
