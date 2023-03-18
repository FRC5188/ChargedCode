package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;

public class CmdArmDisablePID extends CommandBase {
    private Arm _armsubsystem;

    public CmdArmDisablePID(Arm armsubsystem) {
        _armsubsystem = armsubsystem;
    }

    @Override
    public void initialize() {
        _armsubsystem.disablePID();
        _armsubsystem.setShoulderMotorSpeed(0);
        _armsubsystem.setElbowMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
