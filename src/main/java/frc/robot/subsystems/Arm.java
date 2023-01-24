// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  WPI_TalonFX _shoulderMotor;
  WPI_TalonFX _elbowMotor;
  Solenoid _wristSolenoid;
  AnalogInput _elbowPotentiometer;
  AnalogInput _shoulderPotentiometer;
  public enum wristPosition { Parallel, Perpendicular}

  // Arm PID controllers
  private PIDController _shoulderMotorPID;
  private PIDController _elbowMotorPID;

  // internal PID constants
  private double _shoulderMotorPIDMaxSpeed;
  private double _elbowMotorPIDMaxSpeed;

  private double ShoulderMotorkP = 0.0;
  private double ShoulderMotorkI = 0.0;
  private double ShoulderMotorkD = 0.0;
  private double ShoulderMotorkTolerance = 0.0;

  private double ElbowMotorkP = 0.0;
  private double ElbowMotorkI = 0.0;
  private double ElbowMotorkD = 0.0;
  private double ElbowMotorkTolerance = 0.0;

  public Arm() {
    _shoulderMotor = new WPI_TalonFX(Constants.CanIDs.ARM_SHOULDER_MOTOR_CANID);
    _elbowMotor = new WPI_TalonFX(Constants.CanIDs.ARM_ELBOW_MOTOR_CANID);
    _wristSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.WRIST_SOLENOID_PORT);
    _elbowPotentiometer = new AnalogInput(Constants.Aio.ELBOW_PORT_POT);
    _shoulderPotentiometer = new AnalogInput(Constants.Aio.SHOULDER_PORT_POT);

    //Create PID controllers
    _shoulderMotorPID = new PIDController(ShoulderMotorkP, ShoulderMotorkI, ShoulderMotorkD);
    _shoulderMotorPID.setTolerance(ShoulderMotorkTolerance);
    
    _elbowMotorPID = new PIDController(ElbowMotorkP, ElbowMotorkI, ElbowMotorkD);
    _elbowMotorPID.setTolerance(ElbowMotorkTolerance);
  }

/**
     * Initializes the PID controller for moving the shoulder motor
     * @param setpoint setpoint of the shoulder motor
     * @param maxSpeed the max speed of motor, in percent output
     */
    public void shoulderMotorPIDInit(double setpoint,  double maxSpeed) {
      _shoulderMotorPID.setSetpoint(setpoint);
      _shoulderMotorPIDMaxSpeed = maxSpeed;
  }


  /**
     * Initializes the PID controller for moving the elbow motor
     * @param setpoint setpoint of the elbow motor
     * @param maxSpeed the max speed of motor, in percent output
     */
    public void elbowMotorPIDInit(double setpoint,  double maxSpeed) {
      _elbowMotorPID.setSetpoint(setpoint);
      _elbowMotorPIDMaxSpeed = maxSpeed;
  }

  public void setShoulderMotorSpeed(double speed){
    _shoulderMotor.set(speed);
  }

  public void setElbowMotorSpeed(double speed){
    _elbowMotor.set(speed);
  }

  public void setWristPosition(wristPosition position){
    _wristSolenoid.set(position == wristPosition.Parallel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public int getShoulderPotPos(){
    return _shoulderPotentiometer.getAverageValue();
  }
  public int getElbowPotPos(){
    return _elbowPotentiometer.getAverageValue();
  }


  

  /**
     * Executes the shoulder motor PID controller
     */
    public void shoulderMotorPIDExec() {
      double position = getShoulderPotPos();
      double power = _shoulderMotorPID.calculate(position) * _shoulderMotorPIDMaxSpeed;

      setShoulderMotorSpeed(power);
  }

   /**
     * Executes the elbow motor PID controller
     */
    public void elbowMotorPIDExec() {
      double position = getElbowPotPos();
      double power = _elbowMotorPID.calculate(position) * _elbowMotorPIDMaxSpeed;
    
      setElbowMotorSpeed(power);
  }
  

  /** Checks if the joint motors are at their setpoints **/
  public boolean atUpperJointPIDSetpoint() {
    return _shoulderMotorPID.atSetpoint();
}

public boolean atLowerJointPIDSetpoint() {
  return _elbowMotorPID.atSetpoint();
}
}
