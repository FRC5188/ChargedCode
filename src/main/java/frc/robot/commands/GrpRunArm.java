// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrpRunArm extends SequentialCommandGroup {
  /** Creates a new GrpRunArm. */
  private ArmPosition _setpoint;
  private Arm _armSubsystem;

  public GrpRunArm() {
     _armSubsystem = new Arm();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }

  public boolean isStowed() {
    return (_armSubsystem.checkArmPosition() == ArmPosition.Stored);
  }
}
