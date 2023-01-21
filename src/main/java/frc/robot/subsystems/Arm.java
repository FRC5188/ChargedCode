// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  WPI_TalonFX _shoulderMotor;
  WPI_TalonFX _elbowMotor;
  public Arm() {
    _shoulderMotor = new WPI_TalonFX(Constants.CanIDs.ARM_SHOULDER_MOTOR_CANID);
    _elbowMotor = new WPI_TalonFX(Constants.CanIDs.ARM_ELBOW_MOTOR_CANID);

  }

  public void setShoulderMotorSpeed(double speed){
    _shoulderMotor.set(speed);
  }

  public void setElbowMotorSpeed(double speed){
    _elbowMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
