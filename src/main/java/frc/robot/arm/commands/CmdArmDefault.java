package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;

public class CmdArmDefault extends CommandBase {
    private Arm _armSubsystem;

    public CmdArmDefault(Arm armSubsystem) {
        _armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {
        this._armSubsystem.enablePID();
    }

    @Override
    public void execute() {
        //  if(this._armSubsystem.isPIDEnabled()){
            _armSubsystem.execPIDs();
            if (_armSubsystem.atFinalPosition())
                _armSubsystem.setCurrentPosition(_armSubsystem.getTargetArmPosition());
        // }
        // else {
        //     System.out.println("[WARNING]: PID has been disabled.");
        // }
    }

    @Override
    public void end(boolean interrupted) {
        _armSubsystem.setShoulderMotorSpeed(0);
        _armSubsystem.setElbowMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
