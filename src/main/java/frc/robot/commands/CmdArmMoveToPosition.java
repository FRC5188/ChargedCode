package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class CmdArmMoveToPosition extends CommandBase {
    private Arm _armSubsystem;
    private ArmPosition _armPosition;

    public CmdArmMoveToPosition(Arm armSubsystem, ArmPosition armPosition){
        // Setting the local variable to the parameter passed in.        
        this._armSubsystem = armSubsystem;
        this._armPosition = armPosition;
        // Adding arm subsystems to the requirements for this command. 
        this.addRequirements(this._armSubsystem);
    }

    @Override
    public void initialize(){
        this._armSubsystem.moveArmToPositionInit(this._armPosition);
    }

    @Override
    public void execute(){
        this._armSubsystem.moveArmToPositionExec();
    }

    @Override
    public boolean isFinished() {
        return this._armSubsystem.isArmAtSetpoint();
    }
}