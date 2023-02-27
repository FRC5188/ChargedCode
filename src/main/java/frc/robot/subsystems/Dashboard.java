// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase {
  /** Creates a new Dashboard. */
  Arm _armSubsystem;
  Drive _driveSubsystem;
  Vision _visionSubsystem;

  public Dashboard(Arm armSubsystem, Drive driveSubsystem, Vision visionSubsystem) {
    _armSubsystem = armSubsystem;
    _driveSubsystem = driveSubsystem;
    _visionSubsystem = visionSubsystem;


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
