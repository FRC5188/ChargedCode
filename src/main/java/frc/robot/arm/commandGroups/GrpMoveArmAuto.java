package frc.robot.arm.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.arm.commands.CmdArmRunIntake;
import frc.robot.arm.commands.CmdArmUpdateGoal;
import frc.robot.arm.commands.CmdArmWaitForArm;

public class GrpMoveArmAuto extends SequentialCommandGroup {  
    
  public GrpMoveArmAuto(Arm armSubsystem, ArmPosition position) {
    this.addRequirements(armSubsystem);

//
    addCommands(
      // Update arm goal to the intermediate position
      new CmdArmUpdateGoal(armSubsystem, armSubsystem.getIntermediatePosition(position)),
      // Wait for us to get there
      new CmdArmWaitForArm(armSubsystem),
      // Now go to actual position
      new CmdArmUpdateGoal(armSubsystem, position),
      // Wait for arm to reach final position
      new CmdArmWaitForArm(armSubsystem),
      // Turn on the intake
      new CmdArmRunIntake(armSubsystem, 0.4)
    );
    }
}

