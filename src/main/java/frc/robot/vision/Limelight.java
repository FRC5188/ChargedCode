// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  // CONSTANTS
  double LIMELIGHT_HEIGHT = 0.0;
  double LIMELIGHT_TO_CENTER_X_OFFSET = 0.0;
  double tolerance = 1.0;


  NetworkTable table;
  double tx;
  double ty;
  double tv;
  long tclass;
  
  public Limelight() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  private double getDistanceAway(){
    return LIMELIGHT_HEIGHT / Math.tan(Math.abs(ty));
  }

  private double getLimelightToCubeHorizontal(){
    return Math.sin(tx)*getDistanceAway();
  }

  public Boolean isRobotAligned(){
    double offset = LIMELIGHT_TO_CENTER_X_OFFSET - getLimelightToCubeHorizontal();
    return (Math.abs(offset) < tolerance);
  }

  @Override
  public void periodic() {
    tx = table.getValue("tx").getDouble();
    ty = table.getValue("ty").getDouble();
    tv = table.getValue("tv").getDouble();
    tclass = table.getValue("tclass").getInteger();
  }
}
