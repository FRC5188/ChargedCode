package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;

public class CmdArmWaitForArm extends CommandBase {
    private Arm _armSubsystem;

    public CmdArmWaitForArm(Arm armSubsystem) {
        _armSubsystem = armSubsystem;
        this.addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Waiting for arm to reach setpoint...");
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Arm at setpoint");
    }

    @Override
    public boolean isFinished() {
        return _armSubsystem.shoulderAtSetpoint() && _armSubsystem.elbowAtSetpoint();
    }
}
