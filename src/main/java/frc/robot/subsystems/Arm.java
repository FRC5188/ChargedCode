// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final double SHOULDER_0_DEGREE_POT_OFFSET = 2310;
    private final double SHOULDER_90_DEGREE_POT_OFFSET = 1956;
    private final double ELBOW_0_DEGREE_POT_OFFSET = 1660;
    private final double ELBOW_neg90_DEGREE_POT_OFFSET = 2020;

    // All in degrees
    private final double SHOULDER_UPPER_SOFT_STOP = 10;
    private final double SHOULDER_LOWER_SOFT_STOP = -10;
    private final double ELBOW_UPPER_SOFT_STOP = 100;
    private final double ELBOW_LOWER_SOFT_STOP = -2;

    // The y,z position of the shoulder joint relative to the floor
    private final double SHOULDER_JOINT_Z_POS = 17; // inches
    private final double SHOULDER_JOINT_Y_POS = -15; // inches

    // arm segments lengths
    private final double SHOULDER_ARM_LENGTH = 28; // inches
    private final double ELBOW_ARM_LENGTH = 28.5; // inches

    /**
     * A list of possible wrist positions.
     * Parallel = the piston is extended and the intake is parallel with the ground
     * Perpendicular = the piston is retracted and the intake is pointing "up"
     */
    public enum WristPosition {
        Parallel,
        Perpendicular
    }

    /**
     * Represents a 2D position in space for the wrist (the joint at the end of the
     * arm) to
     * be in. The setpoint is a combination of y,x, and a wristpositon. UNITS ARE
     * IN INCHES.
     * 
     * @param y             represents how far out and away from the robot the wrist
     *                      is in inches.
     * @param z             represents how far above the ground the robot claw is in
     *                      inches.
     * @param wristPosition represents a wrist state, either parallel or
     *                      perpendicular.
     */
    public class Arm2DPosition {
        // y represents how far away from the robot the claw is
        // z represents how high above the ground the claw is
        // wrist positons represents if the wrist is extended or retracted
        private double y, z;
        private WristPosition wristPosition;

        public Arm2DPosition(double y, double z, WristPosition wristPosition) {
            this.y = y;
            this.z = z;
            this.wristPosition = wristPosition;
        }

        /**
         * Returns the y position of the setpoint, in inches. Y is the distance out and
         * away from the robot.
         * 
         * @return y in inches
         */
        public double gety() {
            return this.y;
        }

        /**
         * Return the z position of the setpoint, in inches. Z is the height above the
         * ground.
         * 
         * @return x in inches
         */
        public double getz() {
            return this.z;
        }

        /**
         * Return the wristposition of the setpoint as a WristPosition enum.
         * 
         * @return wristPosition
         */
        public WristPosition getWristPosition() {
            return this.wristPosition;
        }

    }

    /**
     * A small container to store and pass around the elbow and joint angles.
     * Have not decided what units the angles are yet. -Garrett
     * 
     * @param shoulderJointAngle The angle of the shoulder joint
     * @param elbowJointAngle    The angle of the elbow joint.
     */
    private class ArmJointAngles {
        private double shoulderJointAngle;
        private double elbowJointAngle;

        public double getShoulderJointAngle() {
            return shoulderJointAngle;
        }

        public double getElbowJointAngle() {
            return elbowJointAngle;
        }

        public ArmJointAngles(double shoulderJointAngle, double elbowJointAngle) {
            this.shoulderJointAngle = shoulderJointAngle;
            this.elbowJointAngle = elbowJointAngle;
        }
    }

    /**
     * Represents the preprogrammed setpoints that the arm should be in during a
     * match.
     */
    public enum ArmPosition {
        Stored, // used while driving across the field or starting a match
        GroundPickUp,
        HighCube,
        HighCone,
        LoadStationPickUp,
        LowScore,
        MiddleCone,
        MiddleCube,
        IntermediatePickup,
        IntermediateScoring,
        CurrentPosition
    }

    ArmPosition _currentArmPos;

    /**
     * constants for the above defined in the ArmPosition Enum.
     * This is the place to make edits to setpoints.
     */
    private final double STORED_Y_POS = 0.0;
    private final double STORED_X_POS = 0.0;
    private final WristPosition STORED_WRIST_POS = WristPosition.Parallel;
    private final Arm2DPosition STORED_SETPOINT = new Arm2DPosition(STORED_Y_POS,
            STORED_X_POS,
            STORED_WRIST_POS);

    private final double GROUND_PICKUP_Y_POS = 0.0;
    private final double GROUND_PICKUP_X_POS = 0.0;
    private final WristPosition GROUND_PICKUP_WRIST_POS = WristPosition.Parallel;
    private final Arm2DPosition GROUND_PICKUP_SETPOINT = new Arm2DPosition(GROUND_PICKUP_Y_POS,
            GROUND_PICKUP_X_POS,
            GROUND_PICKUP_WRIST_POS);

    private final double HIGH_CUBE_Y_POS = 0.0;
    private final double HIGH_CUBE_X_POS = 0.0;
    private final WristPosition HIGH_CUBE_WRIST_POS = WristPosition.Parallel;
    private final Arm2DPosition HIGH_CUBE_SETPOINT = new Arm2DPosition(HIGH_CUBE_Y_POS,
            HIGH_CUBE_X_POS,
            HIGH_CUBE_WRIST_POS);

    private final double HIGH_CONE_Y_POS = 0.0;
    private final double HIGH_CONE_X_POS = 0.0;
    private final WristPosition HIGH_CONE_WRIST_POS = WristPosition.Parallel;
    private final Arm2DPosition HIGH_CONE_SETPOINT = new Arm2DPosition(HIGH_CONE_Y_POS,
            HIGH_CONE_X_POS,
            HIGH_CONE_WRIST_POS);

    private final double LOAD_STATION_PICKUP_Y_POS = 0.0;
    private final double LOAD_STATION_PICKUP_X_POS = 0.0;
    private final WristPosition LOAD_STATION_PICKUP_WRIST_POS = WristPosition.Parallel;
    private final Arm2DPosition LOAD_STATION_PICKUP_SETPOINT = new Arm2DPosition(LOAD_STATION_PICKUP_Y_POS,
            LOAD_STATION_PICKUP_X_POS,
            LOAD_STATION_PICKUP_WRIST_POS);

    private final double LOW_SCORE_Y_POS = 0.0;
    private final double LOW_SCORE_X_POS = 0.0;
    private final WristPosition LOW_SCORE_WRIST_POS = WristPosition.Parallel;
    private final Arm2DPosition LOW_SCORE_SETPOINT = new Arm2DPosition(LOW_SCORE_Y_POS,
            LOW_SCORE_X_POS,
            LOW_SCORE_WRIST_POS);

    private final double MIDDLE_CONE_Y_POS = 0.0;
    private final double MIDDLE_CONE_X_POS = 0.0;
    private final WristPosition MIDDLE_CONE_WRIST_POS = WristPosition.Parallel;
    private final Arm2DPosition MIDDLE_CONE_SETPOINT = new Arm2DPosition(MIDDLE_CONE_Y_POS,
            MIDDLE_CONE_X_POS,
            MIDDLE_CONE_WRIST_POS);

    private final double MIDDLE_CUBE_Y_POS = 0.0;
    private final double MIDDLE_CUBE_X_POS = 0.0;
    private final WristPosition MIDDLE_CUBE_WRIST_POS = WristPosition.Parallel;
    private final Arm2DPosition MIDDLE_CUBE_SETPOINT = new Arm2DPosition(MIDDLE_CUBE_Y_POS,
            MIDDLE_CUBE_X_POS,
            MIDDLE_CUBE_WRIST_POS);
    /** Arm Motor Controllers and Pnuematics. */
    private WPI_TalonFX _shoulderMotor;
    private WPI_TalonFX _elbowMotor;
    private final double MAX_MOTOR_VOLTAGE = 11.5; // May want to adjust -Garrett
    private CANSparkMax _intakeMotor;
    private double _previousIntakeMotorCurrent; // we use this to detect if we have collected a game piece
    private double _intakeMotorCurrent; // this is the current motor current
    private Solenoid _wristSolenoid;

    /** Arm elbow and wrist potentionmeters. Measurers arm angle */
    private AnalogInput _elbowPotentiometer;
    private AnalogInput _shoulderPotentiometer;

    // Arm PID controllers
    private ProfiledPIDController _shoulderMotorPID;
    private ProfiledPIDController _elbowMotorPID;

    /** Arm PID constants */
    // these shold use the maxvelocity below
    // private double _shoulderMotorPIDMaxSpeed = 0.6;
    // private double _elbowMotorPIDMaxSpeed = 0.6;

    // shoulder PID constants
    private final double SHOULD_MOTOR_KP = 0.0;
    private final double SHOLDER_MOTOR_KI = 0.0;
    private final double SHOULDER_MOTOR_KD = 0.0;
    private final double SHOULDER_MOTOR_TOLERANCE = 0.0;

    // shoulder motion profile constraints
    // Velocity is m/s and acceleration is m/s^2
    private final double SHOULDER_MAX_VELOCITY = 70; // max speed that this joint should move at
    private final double SHOULDER_MAX_ACCELERATION = 120; // max acceleration this joint should move at
    private final TrapezoidProfile.Constraints SHOLDER_MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(
            SHOULDER_MAX_VELOCITY,
            SHOULDER_MAX_ACCELERATION);

    // Elbow constants
    private final double ELBOW_MOTOR_KP = 0.0125;
    private final double ELBOW_MOTOR_KI = 0.00;
    private final double ELBOW_MOTOR_KD = 0.00;
    private final double ELBOW_MOTOR_TOLERANCE = 1.0;

    // shoulder motion profile constraints
    // Velocity is m/s and acceleration is m/s^2
    private final double ELBOW_MAX_VELOCITY = 70; // max speed that this joint should move at
    private final double ELBOW_MAX_ACCELERATION = 60; // max acceleration this joint should move at
    private final TrapezoidProfile.Constraints ELBOW_MOTION_PORFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(
            ELBOW_MAX_VELOCITY,
            ELBOW_MAX_ACCELERATION);

    public Arm() {
        // Motor Controllers and pnuematics
        _shoulderMotor = new WPI_TalonFX(Constants.CanIDs.ARM_SHOULDER_MOTOR_CANID);
        _elbowMotor = new WPI_TalonFX(Constants.CanIDs.ARM_ELBOW_MOTOR_CANID);
        _wristSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.PHPorts.WRIST_SOLENOID_PORT);

        // Set the motors to adjust their output based on battery voltage
        _shoulderMotor.configVoltageCompSaturation(this.MAX_MOTOR_VOLTAGE);
        _shoulderMotor.enableVoltageCompensation(true);
        _elbowMotor.configVoltageCompSaturation(this.MAX_MOTOR_VOLTAGE);
        _elbowMotor.enableVoltageCompensation(true);

        // set motor breaking
        _shoulderMotor.setNeutralMode(NeutralMode.Brake);
        _elbowMotor.setNeutralMode(NeutralMode.Brake);

        // Invert shoulder
        _shoulderMotor.setInverted(InvertType.None);
        _elbowMotor.setInverted(InvertType.None);

        // Claw
        _previousIntakeMotorCurrent = 0;
        _intakeMotorCurrent = 0;
        _intakeMotor = new CANSparkMax(Constants.CanIDs.CLAW_INTAKE_MOTOR_CANID, MotorType.kBrushless);

        // Potentiometer
        _elbowPotentiometer = new AnalogInput(Constants.AIO.ELBOW_PORT_POT);
        _elbowPotentiometer.setAverageBits(2);
        _elbowPotentiometer.setOversampleBits(0);
        _shoulderPotentiometer = new AnalogInput(Constants.AIO.SHOULDER_PORT_POT);
        _shoulderPotentiometer.setAverageBits(2);
        _shoulderPotentiometer.setOversampleBits(0);

        // Create Sholder PID controllers
        _shoulderMotorPID = new ProfiledPIDController(this.SHOULD_MOTOR_KP,
                this.SHOLDER_MOTOR_KI,
                this.SHOULDER_MOTOR_KD,
                this.SHOLDER_MOTION_PROFILE_CONSTRAINTS);
        _shoulderMotorPID.setTolerance(this.SHOULDER_MOTOR_TOLERANCE);

        // create elbow PID controller
        _elbowMotorPID = new ProfiledPIDController(this.ELBOW_MOTOR_KP,
                this.ELBOW_MOTOR_KI,
                this.ELBOW_MOTOR_KD,
                this.ELBOW_MOTION_PORFILE_CONSTRAINTS);
        _elbowMotorPID.setTolerance(this.ELBOW_MOTOR_TOLERANCE);

        this.updateShuffleBoard();

        _currentArmPos = ArmPosition.Stored;
    }

    public void setElbowBrakeMode(NeutralMode mode) {
        this._elbowMotor.setNeutralMode(mode);
    }

    private void updateShuffleBoard() {
        SmartDashboard.putNumber("Elbow Angle", this.getElbowJointAngle());
        SmartDashboard.putNumber("Sholder Angle", this.getShoulderJointAngle());
        SmartDashboard.putNumber("Sholder Angle Setpoint", this.getShoulderSetpoint());
        SmartDashboard.putNumber("Elbow Angle Setpoint", this.getElbowSetpoint());
        SmartDashboard.putNumber("Elbow Pot", this.getElbowPotPos());
        SmartDashboard.putNumber("Elbow Motor Output", this._elbowMotor.get());
        SmartDashboard.putNumber("Shoulder Motor Output", this._shoulderMotor.get());

        // System.out.println("ARM ANGLES: Elbow: " + this.getElbowJointAngle() + " Shoulder: " + this.getShoulderJointAngle());
        // System.out.println("Elbow setpoint: " + this.getElbowSetpoint());
         System.out.println("ELBOW POT: " + this.getElbowPotPos());

    }

    private double getElbowSetpoint() {
        // TODO: implement this
        return 0;
    }

    private double getShoulderSetpoint() {
        // TODO: implement this
        return 0;
    }

    /**
     * Sets the arm position
     * 
     * @param position Position to set the arm wrist to.
     */
    public void setWristPosition(WristPosition position) {
        _wristSolenoid.set(position == WristPosition.Parallel);
    }

    /**
     * @return the current position of the wrist
     */
    public WristPosition getWristPosition() {
        return _wristSolenoid.get() ? WristPosition.Parallel : WristPosition.Perpendicular;
    }

    /**
     * Initializes the PID controller for moving the shoulder motor.
     * 
     * This method takes a setpoint for the arm wrist from the this.setpoint class.
     * It then
     * calculates what elbow and shoulder angles are needed to reach the provided
     * setpoint
     * and set its PID controller for that setpoint.
     * 
     * This method may eventually need to do other work.
     * 
     * @param setpoint Setpoint of the wrist of the arm.
     */
    public void shoulderMotorPIDInit(ArmPosition armPosition) {
        _currentArmPos = armPosition;
        Arm2DPosition setpoint;
        switch (armPosition) {
            case GroundPickUp:
                setpoint = GROUND_PICKUP_SETPOINT;
                break;
            case HighCone:
                setpoint = HIGH_CONE_SETPOINT;
                break;
            case HighCube:
                setpoint = HIGH_CUBE_SETPOINT;
                break;
            case LoadStationPickUp:
                setpoint = LOAD_STATION_PICKUP_SETPOINT;
                break;
            case LowScore:
                setpoint = LOW_SCORE_SETPOINT;
                break;
            case MiddleCone:
                setpoint = MIDDLE_CONE_SETPOINT;
                break;
            case MiddleCube:
                setpoint = MIDDLE_CUBE_SETPOINT;
                break;
            case Stored:
                setpoint = STORED_SETPOINT;
                break;
            default:
                // If we hit default that means we don't know what position we are in
                // So we just wanna stay put until we get a new position
                System.out.println(
                        "PASSED A ARM POSITION THAT DID NOT MATCH ANY PRESET. Defaulting to setting the setpint to the current arm position.");
                setpoint = this.getArm2DPosition();
                break;
        }
      
      // TODO: implement this
      // calculate joint angles
      // set pid goal
        
    }

    /**
     * This function will do the math to calculate what combination of elbow and
     * wrist angles
     * will put the wrist in the desired 2D postion
     * 
     * @param arm2DPosition A Arm2DPosition object representing the 2D point in
     *                      space we want the wrist
     *                      to be in. This also includes a wrist state but is
     *                      unused.
     * 
     * @return An ArmJointAngles object representing a shoulder and elbow angle
     *         which yields
     *         the desired wrist 2D position.
     */
    private ArmJointAngles jointAnglesFrom2DPose(Arm2DPosition arm2DPosition) {
        // q2 = elbow angle (upper joint). note the angle is relative to the shoulder
        // angle in this form.
        // See this link
        // https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
        // we will convert it to be relative to the ground after
        // q1 = shoulder angle, a1 = shoulder arm length, a2 = elbow arm length
        // this math uses y as up and x as "out". But we defined Z as up and y as out in
        // our robot positions

        // Note: if you got to 5:13 in the video linked above you can see the math used
        // for this.
        // Note 2: There are exactly 2 solutions of arm angles to reach any given point.
        // We choose to
        // use the solution on the right side because they will put the shoulder joint
        // in the
        // most upright position. This is valuable because that is closest to our
        // "stored"
        // position and means we can quickly move back to having the arm inside the
        // robot frame.
        // Note 3: Here is a the post on CD that gave this info. This team is doing cool
        // stuff!
        // https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2023-build-thread/420691/58

        // NOTE:
        // remember the wrist position is the combination of the arm angles plus the
        // position of shoulder joint!
        // So the arm2dpose.y = arm_kinematics + sholder_start_y and arm2dpose.z =
        // arm_kinematics + shoulder_start_z
        // reworked, arm_kinematics = arm2dpose.y - shoulders_start_y

        // NOTE: 2/13/23 garrett
        // Just using the math above did not work. Not sure why. But I followed along
        // the vidoe and redid the math myself and
        // got a new set of equations. Its very similar to the first set though. (which
        // makes sense) I need to find a place to
        // upload my math and steps I took. Its still only basic geometry and trig. Most
        // high school sphomomores can do it.
        // for now, see zoe or garrett for the math. it should be in slack. The math
        // gives the elbow angle relative the axis
        // and not relative to the shoulder.

        // NOTE: 2/13/23 garrett
        // As the math currently stands there are some extra variables and steps (like
        // variable t1 and gamma).
        // i have not simplified it yet. I started testing it with this form though and
        // don't want to change it until we test
        // more and confirm it works.

        // NOTE: 2/13/23 garrett
        // While rewriting the math from the previous commit i realize i was missing
        // parentheses on line 419. that might
        // have been the issue... :( Still moving forward with this code for now since I
        // have more confidence in it.
        // Here is the commit I referred to:
        // https://github.com/FRC5188/ChargedCode/commit/a35d8567eb73343b4be02176031213db640c6627

        //TODO implement the arm math

        double bigTriangleHypoSqrd = Math.pow(arm2DPosition.y, 2) + Math.pow(arm2DPosition.z, 2);
        //process of law of cosines
        double angleAlpha = Math.pow(SHOULDER_ARM_LENGTH, 2) + Math.pow(ELBOW_ARM_LENGTH, 2);
        angleAlpha = angleAlpha - bigTriangleHypoSqrd;
        angleAlpha = angleAlpha / (2 * SHOULDER_ARM_LENGTH * ELBOW_ARM_LENGTH);
        angleAlpha = Math.acos(angleAlpha);
        double q2 = angleAlpha * -1;
        double tanOfBeta = (ELBOW_ARM_LENGTH * Math.sin(q2)) / (SHOULDER_ARM_LENGTH + ELBOW_ARM_LENGTH * Math.cos(q2));
        double angleBeta = Math.atan(tanOfBeta);
        double angleGamma = Math.atan(arm2DPosition.z / arm2DPosition.y);
        double q1 = angleGamma + angleBeta;
        return new ArmJointAngles(Math.toDegrees(q1), Math.toDegrees(q2));
    }

    /**
     * Get the current position of the arm in 2d space and return it
     * 
     * @return a Arm2DPosition that represents the wrist position in 2D space
     */
    public Arm2DPosition getArm2DPosition() {
        //TODO implement this
        return this.arm2DPositionFromAngles(getShoulderJointAngle(), getElbowJointAngle(), getWristPosition());
    }

    /**
     * Convert the current pot value of the eblow into an angle.
     * 
     * @return Current elbow joint angle in xx units.
     */
    public double getElbowJointAngle() {
        // TODO: implement this
        return 0;
    }

    /**
     * Convert the current pot value of the shoulder into an angle.
     * 
     * @return Current shoulder joint anlge in xx units.
     */
    public double getShoulderJointAngle() {
        // TODO: implement this
        return 0;
    }

    /**
     * This method will do the math to calculate the wrist position in 2D space from
     * a given shoulder and
     * elbow joint angle. This method still needs a current wristposition passed
     * into it so a proper Arm2DPosition
     * can be returned.
     * 
     * @param currentShoulder The current shoulder angle. Use
     *                        {@link getShoulderJointAngle} and not the pot value.
     * @param currentElbow    the current elbow anlge. Use
     *                        {@link getElbowJointAngle} and not the pot value.
     * @param wristPos        The current wrist position. Use
     *                        {@link getWristPosition}
     * @return A Arm2DPosition that represents the current point in 2D space of the
     *         wrist of the arm, including the wrist state.
     */
    private Arm2DPosition arm2DPositionFromAngles(double currentShoulder, double currentElbow, WristPosition wristPos) {
        
        //y1, z1 are the "coordinates" of the shoulder joint.
        double y1 = this.SHOULDER_JOINT_Y_POS;
        double z1 = this.SHOULDER_JOINT_Z_POS;

        // y2, z2 are the "coordinates" of the elbow joint.
        double y2 = ((Math.cos(Math.toRadians(currentShoulder))) * SHOULDER_ARM_LENGTH) + y1;
        double z2 = ((Math.sin(Math.toRadians(currentShoulder))) * SHOULDER_ARM_LENGTH) + z1;

        // y3, z3 are the "coordinates" of the wrist.
        double y3 = ((Math.cos(Math.toRadians(currentElbow))) * ELBOW_ARM_LENGTH) + y2;
        double z3 = ((Math.sin(Math.toRadians(currentElbow))) * ELBOW_ARM_LENGTH) + z2;

       return new Arm2DPosition(y3, z3, wristPos);
    }

    /**
     * Initializes the PID controller for moving the elbow motor.
     * 
     * This method takes a setpoint for the arm wrist from the this.setpoint class.
     * It then
     * calculates what elbow and shoulder angles are needed to reach the provided
     * setpoint
     * and set its PID controller for that setpoint.
     * 
     * This method may eventually need to do other work.
     * 
     * @param setpoint Setpoint of the arm as a whole.
     */
    public void elbowMotorPIDInit(ArmPosition armPosition) {
        _currentArmPos = armPosition;
        Arm2DPosition setpoint;
        switch (armPosition) {
            case GroundPickUp:
                setpoint = GROUND_PICKUP_SETPOINT;
                break;
            case HighCone:
                setpoint = HIGH_CONE_SETPOINT;
                break;
            case HighCube:
                setpoint = HIGH_CUBE_SETPOINT;
                break;
            case LoadStationPickUp:
                setpoint = LOAD_STATION_PICKUP_SETPOINT;
                break;
            case LowScore:
                setpoint = LOW_SCORE_SETPOINT;
                break;
            case MiddleCone:
                setpoint = MIDDLE_CONE_SETPOINT;
                break;
            case MiddleCube:
                setpoint = MIDDLE_CUBE_SETPOINT;
                break;
            case Stored:
                setpoint = STORED_SETPOINT;
                break;
            default:
                // If we hit default that means we don't know what position we are in
                // So we just wanna stay put until we get a new position
                setpoint = this.getArm2DPosition();
                break;
        }
               // TODO: implement this
        // calculate the joint angles
        // set the pid goal
    }

    /**
     * 
     * @return
     */
    public boolean shoulderMotorPIDIsFinished() {
        //         // TODO: implement this
        return this._shoulderMotorPID.atSetpoint();
    }


    /**
     * 
     * @return
     */
    public boolean elbowMotorPIDIsFinished() {
                // TODO: implement this

        return this._elbowMotorPID.atSetpoint();
    }

    /**
     * Executes the shoulder motor PID controller. This PID controller will handle
     * ramping the PID output and setting any limits.
     */
    public void shoulderMotorPIDExec() {
        // TODO: implement this
        double shoulderMotorPower = this._shoulderMotorPID.calculate(this.getShoulderJointAngle());
        this.setShoulderMotorSpeed(shoulderMotorPower);
    }

    /**
     * Executes the elbow motor PID controller. This PID controller will handle
     * ramping the PID output and setting any limits.
     */
    public void elbowMotorPIDExec() {
        // TODO: implement this
        double elbowMotorPower = this._elbowMotorPID.calculate(this.getElbowJointAngle());
        this.setElbowMotorSpeed(elbowMotorPower);
    }

    /** Checks if the joint motors are at their setpoints **/
    public boolean shoulderAtSetpoint() {
        // TODO: implement this
        return false;
    }

    /**
     * Initializes the PID controller for moving the elbow motor
     * 
     * @param setpointAngle setpoint of the elbow motor
     * @param maxSpeed      the max speed of motor, in percent output
     */
    public void elbowMotorPIDInit(double setpointAngle) {
                // TODO: implement this
    }

    public void shoulderMotorPIDInit(double setpointAngle) {
        _shoulderMotorPID.setGoal(setpointAngle);
    }

    /** Checks if the joint motors are at their setpoints **/
    public boolean elbowAtSetpoint() {
        // TODO: implement this
        return false;
    }

    public void setShoulderMotorSpeed(double speed) {
                // TODO: implement this

        //If current angle is above upper soft stop and is still traveling up, stop.
        if (getShoulderJointAngle() > SHOULDER_UPPER_SOFT_STOP && speed > 0) {
            speed = 0;
        }

        //If current angle is below lower soft stop and is still traveling down, stop.
        else if (getShoulderJointAngle() < SHOULDER_LOWER_SOFT_STOP && speed < 0) {
            speed = 0;
        }
        SmartDashboard.putNumber("Shoulder speed", speed);
        //_shoulderMotor.set(speed);
    }

    public void setElbowMotorSpeed(double speed) {
               // TODO: implement this
                
        //If current angle is above upper soft stop and is still traveling up, stop.
        if (getElbowJointAngle() > ELBOW_UPPER_SOFT_STOP && speed > 0) {
            speed = 0;
        }
        
        //If current angle is below lower soft stop and is still traveling down, stop.
        else if (getElbowJointAngle() < ELBOW_LOWER_SOFT_STOP && speed < 0) {
            speed = 0;
        }
        SmartDashboard.putNumber("Elbow speed", speed);        
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
     * 
     * @param speed between -1.0 and 1.0
     */
    public void setIntakeMotorSpeed(double speed) {
        _intakeMotor.set(speed);
    }

    /*
     * Finds the difference in the current intake motor current and the previous
     * one, which are updated in the periodic function
     * 
     * @return the difference in the current intake motor current and the previously
     * recorded one
     */
    public double getChangeInIntakeMotorCurrent() {
        return _intakeMotorCurrent - _previousIntakeMotorCurrent;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        _previousIntakeMotorCurrent = _intakeMotorCurrent;
        _intakeMotorCurrent = _intakeMotor.getOutputCurrent();

        // System.out.println("Shoulder: " + this.getShoulderPotPos() + " Elbow: " +
        // this.getElbowPotPos());
        // System.out.println("Shoulder Angle: " + this.getShoulderJointAngle() + "
        // Elbow Angle: " + this.getElbowJointAngle());
        // ArmJointAngles angles = this.jointAnglesFrom2DPose(new Arm2DPosition(10, 30,
        // GROUND_PICKUP_WRIST_POS));

        // System.out.println("Setpoint: (" + angles.shoulderJointAngle + ", " +
        // angles.elbowJointAngle + ") Shoulder: " + this.getShoulderJointAngle() + "
        // Elbow: " + this.getElbowJointAngle());

        this.updateShuffleBoard();
    }

    public void setWristPosition(ArmPosition armPosition) {
        _currentArmPos = armPosition;
        // Sets Wrist Position based off of arm position
        switch (armPosition) {
            case GroundPickUp:
                setWristPosition(GROUND_PICKUP_WRIST_POS);
                break;
            case HighCone:
                setWristPosition(HIGH_CONE_WRIST_POS);
                break;
            case HighCube:
                setWristPosition(HIGH_CUBE_WRIST_POS);
                break;
            case LoadStationPickUp:
                setWristPosition(LOAD_STATION_PICKUP_WRIST_POS);
                break;
            case LowScore:
                setWristPosition(LOW_SCORE_WRIST_POS);
                break;
            case MiddleCone:
                setWristPosition(MIDDLE_CONE_WRIST_POS);
                break;
            case MiddleCube:
                setWristPosition(MIDDLE_CUBE_WRIST_POS);
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
                return getWristPosition() == HIGH_CONE_WRIST_POS;
            case HighCube:
                return getWristPosition() == HIGH_CUBE_WRIST_POS;
            case LoadStationPickUp:
                return getWristPosition() == LOAD_STATION_PICKUP_WRIST_POS;
            case LowScore:
                return getWristPosition() == LOW_SCORE_WRIST_POS;
            case MiddleCone:
                return getWristPosition() == MIDDLE_CONE_WRIST_POS;
            case MiddleCube:
                return getWristPosition() == MIDDLE_CUBE_WRIST_POS;
            case Stored:
                return getWristPosition() == STORED_WRIST_POS;
            default:
                return false;

        }
    }

    public void SetArmGoal(ArmPosition armGoal) {

        Arm2DPosition setpoint;
        switch (armGoal) {
            case GroundPickUp:
                setpoint = GROUND_PICKUP_SETPOINT;
                break;
            case HighCone:
                setpoint = HIGH_CONE_SETPOINT;
                break;
            case HighCube:
                setpoint = HIGH_CUBE_SETPOINT;
                break;
            case LoadStationPickUp:
                setpoint = LOAD_STATION_PICKUP_SETPOINT;
                break;
            case LowScore:
                setpoint = LOW_SCORE_SETPOINT;
                break;
            case MiddleCone:
                setpoint = MIDDLE_CONE_SETPOINT;
                break;
            case MiddleCube:
                setpoint = MIDDLE_CUBE_SETPOINT;
                break;
            case Stored:
                setpoint = STORED_SETPOINT;
                break;
            default:
                setpoint = this.getArm2DPosition();
                break;
        }
        ArmJointAngles angles = this.jointAnglesFrom2DPose(setpoint);
        _shoulderMotorPID.setGoal(angles.shoulderJointAngle);
        _elbowMotorPID.setGoal(angles.elbowJointAngle);
    }
    public ArmPosition getIntermediateFromPos(ArmPosition position) {
        ArmPosition interPos = ArmPosition.CurrentPosition;

       
        if (_currentArmPos == ArmPosition.Stored) {
            switch (position) {
                case GroundPickUp:
                    interPos = ArmPosition.IntermediatePickup;
                    break;
                case Stored:
                    interPos = ArmPosition.CurrentPosition;
                    break;
                default:
                    interPos = ArmPosition.IntermediateScoring;
                    break;
            }
        }


        if (_currentArmPos == ArmPosition.GroundPickUp) {
            switch (position) {
                case LowScore:
                    interPos = ArmPosition.IntermediatePickup;
                case GroundPickUp:
                    interPos = ArmPosition.CurrentPosition;
                    break;
                default:
                    interPos = ArmPosition.IntermediatePickup;
                    break;
            }
        }

        if (_currentArmPos == ArmPosition.LowScore) {
            switch (position) {
                case LowScore:
                    interPos = ArmPosition.CurrentPosition;
                case GroundPickUp:
                    interPos = ArmPosition.IntermediatePickup;
                    break;
                default:
                    interPos = ArmPosition.IntermediatePickup;
                    break;
            }
        }

        return interPos;
    }
}
