package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Arm2DPosition;
import frc.robot.subsystems.Arm.ArmJointAngles;

public class GrpMoveArmToPositionManual extends SequentialCommandGroup{
    private final double conversionY = 12; // scaling factor
    private final double conversionZ = 12;
    private Arm _armSubsystem;
  //
  
    public GrpMoveArmToPositionManual(Arm armSubsystem, DoubleSupplier ySlider, DoubleSupplier zSlider) {
      _armSubsystem = armSubsystem;
      this.addRequirements(_armSubsystem);
      Arm2DPosition _currentPosition = _armSubsystem.getArm2DPosition();
      double _newYPosition = _currentPosition.gety() + ySlider.getAsDouble() * conversionY;
      double _newZPosition = _currentPosition.getz() + zSlider.getAsDouble() * conversionZ;
      Arm2DPosition _goalPosition = new Arm2DPosition(_newYPosition, _newZPosition, _armSubsystem.getWristPosition());
  //
      System.out.println("Goal position updated to: " + _goalPosition);
      ArmJointAngles goalAngles = _armSubsystem.jointAnglesFrom2DPose(_goalPosition);
      System.out.println("Goal angles: " + goalAngles.getShoulderJointAngle() + ", " + goalAngles.getElbowJointAngle());

      addCommands(
        // Now go to actual position
        new CmdArmUpdateGoalManual(_armSubsystem, _goalPosition),
        // Turn on the intake if needed
        new CmdArmRunIntakeByPosition(_armSubsystem, _armSubsystem.getCurrentArmPosition(), -0.4)
        //
        
      );
  
    }
  
}
