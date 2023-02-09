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

    /**
         * A list of possible wrist positions.
         * Parallel = the piston is extended and the intake is parallel with the ground
         * Perpendicular = the piston is retracted and the intake is pointing "up"
     */
    public enum WristPosition{
        Parallel,
        Perpendicular
    }
   
    /**
     * Represents a setpoint for the wrist (the joint at the end of the arm) to
     * be in. The setpoint is a combination of y,x, and a wristpositon. UNITS ARE
     * IN INCHES.
     * 
     * @param y represents how far out and away from the robot the wrist is in inches. 
     * @param z represents how far above the ground the robot claw is in inches.
     * @param wristPosition represents a wrist state, either parallel or perpendicular.
     */
    public class Setpoint{
        // y represents how far away from the robot the claw is
        // z represents how high above the ground the claw is
        // wrist positons represents if the wrist is extended or retracted
        private double y, z;
        private WristPosition wristPosition;
        public Setpoint(double y, double z, WristPosition wristPosition){
            this.y = y;
            this.z = z;
            this.wristPosition = wristPosition;
        }

        /**
         * Returns the y position of the setpoint, in inches. Y is the distance out and away from the robot.
         * @return y in inches
         */
        public double gety(){
            return this.y;
        }
        /**
         * Return the z position of the setpoint, in inches. Z is the height above the ground.
         * @return x in inches
         */
        public double getz(){
            return this.z;
        }
        /**
         * Return the wristposition of the setpoint as a WristPosition enum.
         * @return wristPosition
         */
        public WristPosition geWristPosition(){
            return this.wristPosition;
        }

       }


    /**
     * Represents the preprogrammed setpoints that the arm should be in during a match.
     */
    public enum ArmPosition{
        Stored, // used while driving across the field or starting a match
        GroundPickUp,
        HighCube, 
        HighCone,
        LoadStationPickUp,
        LowScore,
        MiddleCone,
        MiddleCube
    }

    private final double GROUND_PICKUP_Y_POS = 0.0;
    private final double GROUND_PICKUP_X_POS = 0.0;
    private final WristPosition GROUND_PICKUP_WRIST_POS = WristPosition.Parallel;
    private final Setpoint GROUND_PICKUP_SETPOINT = new Setpoint(GROUND_PICKUP_Y_POS,
                                                                GROUND_PICKUP_X_POS,
                                                                GROUND_PICKUP_WRIST_POS);


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
    
    private int _previousIntakeMotorCurrent;
    private CANSparkMax _intakeMotor;

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
    public void elbowMotorPIDInit(double setpoint) {
        _elbowMotorPID.setSetpoint(setpoint);
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
