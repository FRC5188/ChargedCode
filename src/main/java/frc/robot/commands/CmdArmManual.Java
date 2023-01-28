package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Arm;

/**
 * {@summary} Takes in joystick values from the operator controller and set it to the shoulder and elbow. 
 */
public class CmdArmManual extends CommandBase {
    Arm _armSubsytem;
    // DoubleSupplier passes a method that is run when getAsDouble() is called. Dependency Injection????
    DoubleSupplier _upperJointSupplier;
    DoubleSupplier _lowerJointSupplier;

    public CmdArmManual(Arm armSubsystem, DoubleSupplier upperJointSupplier, DoubleSupplier lowerJointSupplier){
        this._armSubsytem = armSubsystem;
        this._upperJointSupplier = upperJointSupplier;
        this._lowerJointSupplier = lowerJointSupplier;
        // Add the required subsytems. 
        this.addRequirements(_armSubsytem);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        // Set the values give from the joystick to the shoulder and elbow. 
        this._armSubsytem.setShoulderMotorSpeed(_lowerJointSupplier.getAsDouble());
        this._armSubsytem.setElbowMotorSpeed(_upperJointSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}