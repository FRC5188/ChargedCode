package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Arm2DPosition;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Arm.WristPosition;
import java.util.function.DoubleSupplier;

public class CmdArmDefault extends CommandBase {
    private Arm _armSubsystem;
    private double _shoulderGoal;
    private double _elbowGoal;

    public final DoubleSupplier _mToggle;

    public CmdArmDefault(Arm armSubsystem, DoubleSupplier toggle) {
        _armSubsystem = armSubsystem;
        _mToggle = toggle;
    }

    @Override
    public void initialize() {
      
    }

    @Override
    public void execute() {
        _armSubsystem.shoulderMotorPIDExec();
        _armSubsystem.elbowMotorPIDExec();
        Arm2DPosition currentPos = _armSubsystem.getArm2DPosition();
        System.out.println("Current arm position is" + currentPos);
        System.out.println("Current shoulder angle: " + _armSubsystem.getShoulderJointAngle());
        System.out.println("Current elbow angle: " + _armSubsystem.getElbowJointAngle());        

        if (_mToggle.getAsDouble() >= 0){
            _armSubsystem.setArmMode(ArmMode.Cone);
        }else {
            _armSubsystem.setArmMode(ArmMode.Cube);
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
