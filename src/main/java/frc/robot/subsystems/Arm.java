// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

    public enum ArmPosition {
        Stored,
        /* GAME PIECE PICKUP */
        LoadStationPickUp,
        GroundPickUp,

        /* GAME PIECE SCORING */
        // High Goal
        HighCone,
        HighCube,
        // Middle Goal
        MiddleCone,
        MiddleCube,
        // Low Goal
        LowScore
    }

    // LOAD STATION
    private final double LOADING_STATION_SHOULDER_POS = -1.0;
    private final double LOADING_STATION_ELBOW_POS = -1.0;
    private final WristPosition LOADING_STATION_WRIST_POS = WristPosition.Parallel;
    // GROUND PICKUP
    private final double GROUND_PICKUP_SHOULDER_POS = -1.0;
    private final double GROUND_PICKUP_ELBOW_POS = -1.0;
    private final WristPosition GROUND_PICKUP_WRIST_POS = WristPosition.Parallel;
    // HIGH GOAL
    private final double HIGH_GOAL_CONE_SHOULDER_POS = -1.0;
    private final double HIGH_GOAL_CONE_ELBOW_POS = -1.0;
    private final WristPosition HIGH_GOAL_CONE_WRIST_POS = WristPosition.Parallel;

    private final double HIGH_GOAL_CUBE_SHOULDER_POS = -1.0;
    private final double HIGH_GOAL_CUBE_ELBOW_POS = -1.0;
    private final WristPosition HIGH_GOAL_CUBE_WRIST_POS = WristPosition.Parallel;
    // MIDDLE GOAL
    private final double MIDDLE_GOAL_CONE_SHOULDER_POS = -1.0;
    private final double MIDDLE_GOAL_CONE_ELBOW_POS = -1.0;
    private final WristPosition MIDDLE_GOAL_CONE_WRIST_POS = WristPosition.Parallel;

    private final double MIDDLE_GOAL_CUBE_SHOULDER_POS = -1.0;
    private final double MIDDLE_GOAL_CUBE_ELBOW_POS = -1.0;
    private final WristPosition MIDDLE_GOAL_CUBE_WRIST_POS = WristPosition.Parallel;
    // LOW GOAL
    private final double LOW_GOAL_SHOULDER_POS = -1.0;
    private final double LOW_GOAL_ELBOW_POS = -1.0;
    private final WristPosition LOW_GOAL_WRIST_POS = WristPosition.Parallel;

    // CLAW
    private CANSparkMax _intakeMotor;
    private double _previousIntakeMotorCurrent;
    private double _intakeMotorCurrent;

    /** Creates a new Arm. */
    private WPI_TalonFX _shoulderMotor;
    private WPI_TalonFX _elbowMotor;
    private Solenoid _wristSolenoid;
    private AnalogInput _elbowPotentiometer;
    private AnalogInput _shoulderPotentiometer;

    // Arm PID controllers
    private PIDController _shoulderMotorPID;
    private PIDController _elbowMotorPID;

    // internal PID constants
    private double _shoulderMotorPIDMaxSpeed = 0.6;
    private double _elbowMotorPIDMaxSpeed = 0.6;

    private double ShoulderMotorkP = 0.0;
    private double ShoulderMotorkI = 0.0;
    private double ShoulderMotorkD = 0.0;
    private double ShoulderMotorkTolerance = 0.0;

    private double ElbowMotorkP = 0.0;
    private double ElbowMotorkI = 0.0;
    private double ElbowMotorkD = 0.0;
    private double ElbowMotorkTolerance = 0.0;

    public enum ArmPositionState {
        Stored,
        /* GAME PIECE PICKUP */
        LoadStationPickUp,
        GroundPickUp,
    }

    public Arm() {
        _shoulderMotor = new WPI_TalonFX(Constants.CanIDs.ARM_SHOULDER_MOTOR_CANID);
        _elbowMotor = new WPI_TalonFX(Constants.CanIDs.ARM_ELBOW_MOTOR_CANID);
        _wristSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.PHPorts.WRIST_SOLENOID_PORT);
        _elbowPotentiometer = new AnalogInput(Constants.AIO.ELBOW_PORT_POT);
        _shoulderPotentiometer = new AnalogInput(Constants.AIO.SHOULDER_PORT_POT);
        // Claw
        _previousIntakeMotorCurrent = 0;
        _intakeMotor = new CANSparkMax(Constants.CanIDs.CLAW_INTAKE_MOTOR_CANID, MotorType.kBrushless);

        // Create PID controllers
        _shoulderMotorPID = new PIDController(Constants.PID.ShoulderMotorkP, Constants.PID.ShoulderMotorkI,
                Constants.PID.ShoulderMotorkD);
        _shoulderMotorPID.setTolerance(Constants.PID.ShoulderMotorkTolerance);

        _elbowMotorPID = new PIDController(ElbowMotorkP, ElbowMotorkI,
                ElbowMotorkD);
        _elbowMotorPID.setTolerance(ElbowMotorkTolerance);  
    }

    public void setWristPosition(WristPosition position) {
        _wristSolenoid.set(position == WristPosition.Parallel);
    }

    public WristPosition getWristPosition() {
        return _wristSolenoid.get() ? WristPosition.Parallel : WristPosition.Perpendicular;
    }

    /**
     * Initializes the PID controller for moving the shoulder motor
     * 
     * @param setpoint setpoint of the shoulder motor
     * @param maxSpeed the max speed of motor, in percent output
     */
    public void shoulderMotorPIDInit(double setpoint) {
        _shoulderMotorPID.setSetpoint(setpoint);
    }
    
    /**
     * Executes the shoulder motor PID controller
     */
    public void shoulderMotorPIDExec() {
        double position = getShoulderPotPos();
        double power = _shoulderMotorPID.calculate(position) * _shoulderMotorPIDMaxSpeed;

        setShoulderMotorSpeed(power);
    }

    /** Checks if the joint motors are at their setpoints **/
    public boolean shoulderAtSetpoint() {
        return _shoulderMotorPID.atSetpoint();
    }

    /**
     * Initializes the PID controller for moving the elbow motor
     * 
     * @param setpoint setpoint of the elbow motor
     * @param maxSpeed the max speed of motor, in percent output
     */
    public void elbowMotorPIDInit(double setpoint) {
        _elbowMotorPID.setSetpoint(setpoint);
    }

    /**
     * Executes the shoulder motor PID controller
     */
    public void elbowMotorPIDExec() {
        double position = getElbowPotPos();
        double power = _elbowMotorPID.calculate(position) * _elbowMotorPIDMaxSpeed;

        setElbowMotorSpeed(power);
    }

    /** Checks if the joint motors are at their setpoints **/
    public boolean elbowAtSetpoint() {
        return _elbowMotorPID.atSetpoint();
    }

    public void setShoulderMotorSpeed(double speed) {
        _shoulderMotor.set(speed);
    }

    public void setElbowMotorSpeed(double speed) {
        _elbowMotor.set(speed);
    }

    public int getShoulderPotPos() {
        return _shoulderPotentiometer.getAverageValue();
    }

    public int getElbowPotPos() {
        return _elbowPotentiometer.getAverageValue();
    }

    public void moveArmToPositionInit(ArmPosition setpoint) {
        // Start with our values being where we currently are
        // That way, if there's an issue, the arm just stays where it is
        double shoulderSetpoint = this.getShoulderPotPos();
        double elbowSetpoint = this.getElbowPotPos();
        WristPosition wristPosition = this.getWristPosition();

        switch (setpoint) {
            case GroundPickUp:
                shoulderSetpoint = GROUND_PICKUP_SHOULDER_POS;
                elbowSetpoint = GROUND_PICKUP_ELBOW_POS;
                wristPosition = GROUND_PICKUP_WRIST_POS;
                break;
            case HighCone:
                break;
            case HighCube:
                break;
            case LoadStationPickUp:
                break;
            case LowScore:
                break;
            case MiddleCone:
                break;
            case MiddleCube:
                break;
            case Stored:
                break;
            default:
                // If we hit default that means we don't know what position we are in
                // So we just wanna stay put until we get a new position
                break;
        }

        this.shoulderMotorPIDInit(shoulderSetpoint);
        this.elbowMotorPIDInit(elbowSetpoint);
        this.setWristPosition(wristPosition);
    }

    public void moveArmToPositionExec() {
        // FILL ME OUT!
    }

    public boolean isArmAtSetpoint() {
        // FILL ME OUT!

        // placeholder
        return true;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        _previousIntakeMotorCurrent = _intakeMotorCurrent;
        _intakeMotorCurrent = _intakeMotor.getOutputCurrent();
    }

    /*
     * Sets the current speed of the intake motor on the claw
     * @param speed between -1.0 and 1.0
     */
    public void setIntakeMotorSpeed(double speed){
        _intakeMotor.set(speed);
    }

    /*
     * Finds the difference in the current intake motor current and the previous one, which are updated in the periodic function
     * @return the difference in the current intake motor current and the previously recorded one
     */
    public double getChangeInIntakeMotorCurrent(){
        return _intakeMotorCurrent - _previousIntakeMotorCurrent;
    }

}
