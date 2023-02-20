package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Arm.WristPosition;

public class CmdArmDefault extends CommandBase {
    private Arm _armSubsystem;
    private double _shoulderGoal;
    private double _elbowGoal;

    public CmdArmDefault(Arm armSubsystem) {
        _armSubsystem = armSubsystem;

        this.addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        // When we restart this comman (after updating goal), we want to go to the current goal
        // _armSubsystem.shoulderPIDSetGoal(_armSubsystem.checkArmPosition());
        // _armSubsystem.elbowPIDSetGoal(_armSubsystem.checkArmPosition());
        // _armSubsystem.setWristPosition(_armSubsystem.checkArmPosition());
        //_armSubsystem.setCurrentArmPosition(_armSubsystem.checkArmPosition());
    }

    @Override
    public void execute() {
        // Run the arm if safe
        // if (_armSubsystem.isSafeForShoulder()) {
        //     System.out.println("Running shoulder");
        //     _armSubsystem.shoulderMotorPIDExec();
        // }

        // if (_armSubsystem.isSafeForElbow()) {
        //     System.out.println("Running elbow");
        //     _armSubsystem.elbowMotorPIDExec();
        // }
            

        // WristPosition pos = _armSubsystem.getDesiredWristPosition();
        // if (_armSubsystem.isSafeForWrist() && pos != _armSubsystem.getWristPosition()) {
        //     System.out.println("Running wrist");
        //     _armSubsystem.setWristPosition(pos);
        // }

        _armSubsystem.shoulderMotorPIDExec();
        _armSubsystem.elbowMotorPIDExec();
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
