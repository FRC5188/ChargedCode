// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    
    // STORED
    private final double STORED_SHOULDER_POS = -1.0;
    private final double STORED_ELBOW_POS = -1.0
    private final WristPosition STORED_WRIST_POS = WristPosition.Parallel;

    //STORE
    private final double STORE_SHOULDER_POS = -1.0;
    private final double STORE_ELBOW_POS = -1.0;
    private final WristPosition STORE_WRIST_POS = WristPosition.Parallel;

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

    private int SHOULDER_UPPER_SOFT_STOP = 1920;
    private int SHOULDER_LOWER_SOFT_STOP = 2150;
    private int ELBOW_UPPER_SOFT_STOP = 2130;
    private int ELBOW_LOWER_SOFT_STOP = 1680;

    public enum ArmPositionState {
        Stored,
        /* GAME PIECE PICKUP */
        LoadStationPickUp,
        GroundPickUp,
    }

    public Arm() {
        _shoulderMotor = new WPI_TalonFX(Constants.CanIDs.ARM_SHOULDER_MOTOR_CANID);
        _elbowMotor = new WPI_TalonFX(Constants.CanIDs.ARM_ELBOW_MOTOR_CANID);
        _shoulderMotor.setNeutralMode(NeutralMode.Brake);
        _shoulderMotor.setInverted(InvertType.InvertMotorOutput);
        _elbowMotor.setNeutralMode(NeutralMode.Brake);

        _wristSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.PHPorts.WRIST_SOLENOID_PORT);

        _elbowPotentiometer = new AnalogInput(Constants.AIO.ELBOW_PORT_POT);
        _shoulderPotentiometer = new AnalogInput(Constants.AIO.SHOULDER_PORT_POT);
        _elbowPotentiometer.setAverageBits(2);
        _elbowPotentiometer.setOversampleBits(0);
        _shoulderPotentiometer.setAverageBits(2);
        _shoulderPotentiometer.setOversampleBits(0);
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
    public void shoulderMotorPIDInit(ArmPosition setpoint) {

        double shoulderSetpoint = this.getShoulderPotPos();

        switch (setpoint) {
            case GroundPickUp:
                shoulderSetpoint = GROUND_PICKUP_SHOULDER_POS;
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
        _shoulderMotorPID.setSetpoint(shoulderSetpoint);
    }

    public void elbowMotorPIDInit(ArmPosition setpoint) {

        double elbowSetpoint = this.getElbowPotPos();

        switch (setpoint) {
            case GroundPickUp:
                elbowSetpoint = GROUND_PICKUP_ELBOW_POS;
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
        _elbowMotorPID.setSetpoint(elbowSetpoint);
    }
    
    public boolean shoulderMotorPIDIsFinished() {
        return _shoulderMotorPID.atSetpoint();
    }

    public boolean elbowMotorPIDIsFinished() {
        return _elbowMotorPID.atSetpoint();
    }

    /**
     * Executes the shoulder motor PID controller
     */
    public void shoulderMotorPIDExec() {
        double position = getShoulderPotPos();
        double power = _shoulderMotorPID.calculate(position) * _shoulderMotorPIDMaxSpeed;
        
        setShoulderMotorSpeed(power);
    }

    public void elbowMotorPIDExec() {
        double position = getElbowPotPos();
        double power = _elbowMotorPID.calculate(position) * _elbowMotorPIDMaxSpeed;
        
        setElbowMotorSpeed(power);
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
    public void elbowMotorPIDInit(ArmPosition setpoint) {
        // Start with our values being where we currently are
            // That way, if there's an issue, the arm just stays where it is
            double elbowSetpoint = this.getElbowPotPos();
    
            switch (setpoint) {
                case GroundPickUp:
                    elbowSetpoint = GROUND_PICKUP_ELBOW_POS;
                    break;
                case HighCone:
                    elbowSetpoint = HIGH_GOAL_CONE_ELBOW_POS;
                    break;
                case HighCube:
                    elbowSetpoint = HIGH_GOAL_CUBE_ELBOW_POS;
                    break;
                case LoadStationPickUp:
                    elbowSetpoint = LOADING_STATION_ELBOW_POS;
                    break;
                case LowScore:
                    elbowSetpoint = LOW_GOAL_ELBOW_POS;
                    break;
                case MiddleCone:
                    elbowSetpoint = MIDDLE_GOAL_CONE_ELBOW_POS;
                    break;
                case MiddleCube:
                    elbowSetpoint = MIDDLE_GOAL_CUBE_ELBOW_POS;
                    break;
                case Stored:
                    elbowSetpoint = STORED_ELBOW_POS;
                    break;
                default:
                    // If we hit default that means we don't know what position we are in
                    // So we just wanna stay put until we get a new position
                    elbowSetpoint = this.getElbowPotPos();
                    break;
            }
        _elbowMotorPID.setSetpoint(elbowSetpoint);
    }

    /** Checks if the joint motors are at their setpoints **/
    public boolean elbowAtSetpoint() {
        return _elbowMotorPID.atSetpoint();
    }

    public void setShoulderMotorSpeed(double speed) {
        if ((this.getShoulderPotPos() < this.SHOULDER_UPPER_SOFT_STOP && speed < 0) || (this.getShoulderPotPos() > this.SHOULDER_LOWER_SOFT_STOP && speed > 0)) {
            _shoulderMotor.set(0);
            return;
        }

        _shoulderMotor.set(speed);
    }

    public void setElbowMotorSpeed(double speed) {
        if ((this.getElbowPotPos() > this.ELBOW_UPPER_SOFT_STOP && speed < 0) || (this.getElbowPotPos() < this.ELBOW_LOWER_SOFT_STOP && speed > 0)) {
            _elbowMotor.set(0);
            return;
        }

        _elbowMotor.set(speed);
    }

    public int getShoulderPotPos() {
        return _shoulderPotentiometer.getAverageValue();
    }

    public int getElbowPotPos() {
        return _elbowPotentiometer.getAverageValue();
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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        _previousIntakeMotorCurrent = _intakeMotorCurrent;
        _intakeMotorCurrent = _intakeMotor.getOutputCurrent();

        System.out.println("Shoulder Pot: " + this.getShoulderPotPos() + " Elbow Pot: " + this.getElbowPotPos());
    }

    public void setWristPosition(ArmPosition armPosition) {
        //Sets Wrist Position based off of arm position
        switch (armPosition) {
            case GroundPickUp:
                setWristPosition(GROUND_PICKUP_WRIST_POS);
                break;
            case HighCone:
                setWristPosition(HIGH_GOAL_CONE_WRIST_POS);
                break;
            case HighCube:
                setWristPosition(HIGH_GOAL_CUBE_WRIST_POS);
                break;
            case LoadStationPickUp:
                setWristPosition(LOADING_STATION_WRIST_POS);
                break;
            case LowScore:
                setWristPosition(LOW_GOAL_WRIST_POS);
                break;
            case MiddleCone:
                setWristPosition(MIDDLE_GOAL_CONE_WRIST_POS);
                break;
            case MiddleCube:
                setWristPosition(MIDDLE_GOAL_CUBE_WRIST_POS);
                break;
            case Stored:

                break;
            default:
                break; 
        }
    }

    public boolean checkWristPosition(ArmPosition positionOfArm) {
        switch (positionOfArm) {
            case GroundPickUp:
                return getWristPosition() == GROUND_PICKUP_WRIST_POS;
            case HighCone:
                return getWristPosition() == HIGH_GOAL_CONE_WRIST_POS;
            case HighCube:
                return getWristPosition() == HIGH_GOAL_CUBE_WRIST_POS;
            case LoadStationPickUp:
                return getWristPosition() == LOADING_STATION_WRIST_POS;
            case LowScore:
                return getWristPosition() == LOW_GOAL_WRIST_POS;
            case MiddleCone:
                return getWristPosition() == MIDDLE_GOAL_CONE_WRIST_POS;
            case MiddleCube:
                return getWristPosition() == MIDDLE_GOAL_CUBE_WRIST_POS;
            case Stored:
                return getWristPosition() == STORE_WRIST_POS; 
            default:
                return false;
             
        }
    }
}
