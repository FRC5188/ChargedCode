// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;

public class JointConfig{
    double mass;
    double length;
    double moi;
    double cgRadius;    
    DCMotor motor;
    public JointConfig(double mass, double length, double moi, double cgRadius, DCMotor motor) {
        this.mass = mass;
        this.length = length;
        this.moi = moi;
        this.cgRadius = cgRadius;
        this.motor = motor;
    }
  }
  