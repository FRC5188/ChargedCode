// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.Drive;
import frc.robot.vision.Limelight;

public class CmdLimelightAlign extends CommandBase {
  private final Drive m_drivetrainSubsystem;
  private final Limelight m_limelightSubsystem;

  /** Creates a new CmdLimelightAlign. */
  public CmdLimelightAlign( Drive drivetrainSubsystem, Limelight limelightSubsystem ) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.m_limelightSubsystem = limelightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_limelightSubsystem.getTx());
    if (m_limelightSubsystem.getTx() <= 0) {
      m_drivetrainSubsystem.drive(new ChassisSpeeds(0, -1, 0));
    
    }
    else if (m_limelightSubsystem.getTx() >=0) {
      m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 1, 0));

    }
    /*
     * 
     * when tx is negative, the robot should move left
     * when tx is positive, the robot should move right
     * 
     * 
     */
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void periodic(){
    System.out.println(m_limelightSubsystem.getTx());
  }
}
