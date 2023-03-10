package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CmdArmSetMode;
import frc.robot.commands.GrpMoveArmToPosition;
import frc.robot.commands.GrpMoveArmToScore;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Arm.ArmPosition;

public class GrpAutoHighCubeAndBalance extends SequentialCommandGroup {
  public GrpAutoHighCubeAndBalance(Drive driveSubsystem, Arm armSubsystem) {

    addCommands(
        // Make sure we are in cube mode
        new CmdArmSetMode(armSubsystem, ArmMode.Cube),
        // Move the arm to high cube
        new GrpMoveArmToPosition(armSubsystem, ArmPosition.High),
        // Score the piece
        new GrpMoveArmToScore(armSubsystem),
        // Drive onto platform
        Autonomous.getPreloadedPathCommand("TEST_Short_Distance", 3, 1.5, driveSubsystem, driveSubsystem::drive)
    );
  }
}
