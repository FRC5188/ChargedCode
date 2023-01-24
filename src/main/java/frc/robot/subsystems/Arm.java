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
    public enum WristPosition {
        Parallel,
        Perpendicular
    }

    // LOAD STATION
    public static final double LOADING_STATION_SHOULDER_POS = -1.0; 
    public static final double LOADING_STATION_ELBOW_POS = -1.0;
    // GROUND PICKUP
    public static final double GROUND_PICKUP_SHOULDER_POS = -1.0;
    public static final double GROUND_PICKUP_ELBOW_POS = -1.0;
    // HIGH GOAL
    public static final double HIGH_GOAL_SHOULDER_POS = -1.0;
    public static final double HIGH_GOAL_ELBOW_POS = -1.0;
    // MIDDLE GOAL
    public static final double MIDDLE_GOAL_SHOULDER_POS = -1.0;
    public static final double MIDDLE_GOAL_ELBOW_POS = -1.0;
    // LOW GOAL
    public static final double LOW_GOAL_SHOULDER_POS = -1.0;
    public static final double LOW_GOAL_ELBOW_POS = -1.0;

    /** Creates a new Arm. */
    private WPI_TalonFX _shoulderMotor;
    private WPI_TalonFX _elbowMotor;
    private Solenoid _wristSolenoid;
    private AnalogInput _elbowPotentiometer;
    private AnalogInput _shoulderPotentiometer;

    // Arm PID controllers
    private PIDController m_shoulderMotorPID;
    private PIDController m_elbowMotorPID;

    // internal constants
    private double m_shoulderMotorPIDMaxSpeed;
    private double m_elbowMotorPIDMaxSpeed;

    public Arm() {
        _shoulderMotor = new WPI_TalonFX(Constants.CanIDs.ARM_SHOULDER_MOTOR_CANID);
        _elbowMotor = new WPI_TalonFX(Constants.CanIDs.ARM_ELBOW_MOTOR_CANID);
        _wristSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.WRIST_SOLENOID_PORT);
        _elbowPotentiometer = new AnalogInput(Constants.AIO.ELBOW_PORT_POT);
        _shoulderPotentiometer = new AnalogInput(Constants.AIO.SHOULDER_PORT_POT);

        // Create PID controllers
        m_shoulderMotorPID = new PIDController(Constants.PID.ShoulderMotorkP, Constants.PID.ShoulderMotorkI,
                Constants.PID.ShoulderMotorkD);
        m_shoulderMotorPID.setTolerance(Constants.PID.ShoulderMotorkTolerance);

        m_elbowMotorPID = new PIDController(Constants.PID.ElbowMotorkP, Constants.PID.ElbowMotorkI,
                Constants.PID.ElbowMotorkD);
        m_elbowMotorPID.setTolerance(Constants.PID.ElbowMotorkTolerance);
    }

    public void setShoulderMotorSpeed(double speed) {
        _shoulderMotor.set(speed);
    }

    public void setElbowMotorSpeed(double speed) {
        _elbowMotor.set(speed);
    }

    public void setWristPosition(WristPosition position) {
        _wristSolenoid.set(position == WristPosition.Parallel);
    }

    public int getShoulderPotPos() {
        return _shoulderPotentiometer.getAverageValue();
    }

    public int getElbowPotPos() {
        return _elbowPotentiometer.getAverageValue();
    }

    /**
     * Initializes the PID controller for moving the shoulder motor
     * 
     * @param setpoint setpoint of the shoulder motor
     * @param maxSpeed the max speed of motor, in percent output
     */
    public void shoulderMotorPIDInit(double setpoint, double maxSpeed) {
        m_shoulderMotorPID.setSetpoint(setpoint);
        m_shoulderMotorPIDMaxSpeed = maxSpeed;
    }

    /**
     * Initializes the PID controller for moving the elbow motor
     * 
     * @param setpoint setpoint of the elbow motor
     * @param maxSpeed the max speed of motor, in percent output
     */
    public void elbowMotorPIDInit(double setpoint, double maxSpeed) {
        m_elbowMotorPID.setSetpoint(setpoint);
        m_elbowMotorPIDMaxSpeed = maxSpeed;
    }

    /**
     * Executes the shoulder motor PID controller
     */
    public void shoulderMotorPIDExec() {
        double position = getShoulderPotPos();
        double power = m_shoulderMotorPID.calculate(position) * m_shoulderMotorPIDMaxSpeed;

        setShoulderMotorSpeed(power);
    }

    /**
     * Executes the elbow motor PID controller
     */
    public void elbowMotorPIDExec() {
        double position = getElbowPotPos();
        double power = m_elbowMotorPID.calculate(position) * m_elbowMotorPIDMaxSpeed;

        setElbowMotorSpeed(power);
    }

    /** Checks if the joint motors are at their setpoints **/
    public boolean atUpperJointPIDSetpoint() {
        return m_shoulderMotorPID.atSetpoint();
    }

    public boolean atLowerJointPIDSetpoint() {
        return m_elbowMotorPID.atSetpoint();
    }

    public void moveArmToPosition(Constants.ArmPosition setpoint) {
        
        double shoulderSetpoint;
        double elbowSetpoint;
        WristPosition wristPosition;

        switch(setpoint) {
            case GroundPickUpCone:

                break;
            case HighCone:
                break;
            case HighCube:
                break;
            case LoadStationPickUp:
                break;
            case LowCone:
                break;
            case LowCube:
                break;
            case MiddleCone:
                break;
            case MiddleCube:
                break;
            case Stored:
                break;
            default:
                break;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
