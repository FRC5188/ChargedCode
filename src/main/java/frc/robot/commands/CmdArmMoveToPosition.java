package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class CmdArmMoveToPosition extends CommandBase {
    private Arm armSubsystem;
    private ArmPosition armPosition;

    public CmdArmMoveToPosition(Arm armSubsystem, ArmPosition armPosition){
        // Setting the local variable to the parameter passed in.        
        this.armSubsystem = armSubsystem;
        this.armPosition = armPosition;
        // Adding arm subsystems to the requirements for this command. 
        this.addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        armSubsystem.moveArmToPositionInit(armPosition);
    }

    @Override
    public void execute(){
        armSubsystem.moveArmToPositionExec();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isArmAtSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}