package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Arm2DPosition;

public class CmdArmDefault extends CommandBase {
    private Arm _armSubsystem;
    private double _shoulderGoal;
    private double _elbowGoal;

    public CmdArmDefault(Arm armSubsystem) {
        _armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(!this._armSubsystem.isPIDEnabled()){
            _armSubsystem.shoulderMotorPIDExec();
            _armSubsystem.elbowMotorPIDExec();
            Arm2DPosition currentPos = _armSubsystem.getArm2DPosition();
        }
        else {
            System.out.println("[WARNING]: PID has been disabled.");
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Turn off motors
        _armSubsystem.setShoulderMotorSpeed(0);
        _armSubsystem.setElbowMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        // This is a default command, so there is no end
        return false;
    }
}
