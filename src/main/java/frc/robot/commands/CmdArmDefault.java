package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Arm2DPosition;


/*
 * Although this is called a "default" command it technically is not the default command for the arm.
 * This command does not require the arm subsystem and therfore does not get interrupted or interrupt other
 * commands that need the arm. We start this command at the beginning of the robot code and it runs forever. 
 * 
 * We treat it as a "default" to decide what to do with the PID loops and other things. This allows us to call
 * commands to update the position and states of the arm and instantly finish while this command keeps running. 
 * 
 * Our arm is ALWAYS running PID (and Feedforward) when enabled. The robot is always tying to hold its position. We
 * simply keep updating what that position should be. 
 * 
 * The only time we disable PID is if the operator presses the disable PID button to save the robot from damaging itself.
 * (we've broken many chains from PID running the arm into the robot)
 */
public class CmdArmDefault extends CommandBase {
    private Arm _armSubsystem;
    private double _shoulderGoal;
    private double _elbowGoal;

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
            _armSubsystem.shoulderMotorPIDExec();
            _armSubsystem.elbowMotorPIDExec();
            if(_armSubsystem.shoulderAtSetpoint() && _armSubsystem.elbowAtSetpoint()){
                _armSubsystem.setCurrentPosition(_armSubsystem.getTargetArmPosition());
            }
            Arm2DPosition currentPos = _armSubsystem.getArm2DPosition();
        // }
        // else {
        //     System.out.println("[WARNING]: PID has been disabled.");
        // }
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
