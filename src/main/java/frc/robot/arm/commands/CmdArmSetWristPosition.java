package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;

public class CmdArmSetWristPosition extends CommandBase {
    private Arm _armSubsystem;

    public CmdArmSetWristPosition(Arm armSubsystem) {
        this._armSubsystem = armSubsystem;

        this.addRequirements(_armSubsystem);
    }

    @Override
    public void initialize() {
        ArmPosition position = _armSubsystem.getCurrentArmPosition();
        _armSubsystem.setWristPosition(position);
        System.out.println("set wrist position to " + position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
