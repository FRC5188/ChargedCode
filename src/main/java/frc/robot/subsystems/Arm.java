// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final double SHOULDER_0_DEGREE_POT_OFFSET = 2217;
    private final double SHOULDER_90_DEGREE_POT_OFFSET = 1868;
    private final double ELBOW_0_DEGREE_POT_OFFSET = 1708;
    private final double ELBOW_neg90_DEGREE_POT_OFFSET = 2060;

    // All in degrees
    private final double SHOULDER_UPPER_SOFT_STOP = 115;
    private final double SHOULDER_LOWER_SOFT_STOP = 5;
    private final double ELBOW_UPPER_SOFT_STOP = 120;
    private final double ELBOW_LOWER_SOFT_STOP = -10;

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
    public static class Arm2DPosition {
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
        public WristPosition geWristPosition() {
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
    public static class ArmJointAngles {
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
        IntermediateScoring,
        IntermediatePickup,
        Middle,
        High
    }

    public enum ArmMode {
        Cone,
        Cube
    }

    /* How to update a setpoint:
     * 1) Get the robot ready. You MUST comment out the lines letting the 
     *    shoulder and elbow move (_shoulderMotor.set for example). 
     *    There should only be 2 spots this happens, but I suggest searching 
     *    for .set( to make sure everything is commented out. You also need to change 
     *    the idle mode to coast instead of brake so you can move the arm. 
     *    That gets handled in the constructor. Do for both joints.
     * 2) Open up shuffleboard. If you go to the smart dashboard tab, you should
     *    see a bunch of numbers. What you are looking for is the ones labeled 
     *    elbow angle and shoulder angle. Those should update as you move the arm.
     * 3) Move to your new position and take note of those angles. They will 
     *    get displayed in the smart dashboard.
     * 4) Change the values in code. Make sure you pick the correct position
     *    and update the numbers for the shoulder and elbow angles you just took
     * 5) Uncomment the stuff you commented out and change the idle mode back
     *    to brake. Go ahead and test. MAKE SURE YOU'RE READY TO DISABLE! 
     */

    private final double STORED_SHOULDER_POS = 81.7;
    private final double STORED_ELBOW_POS = -1.3;

    private final double MID_CONE_SHOULDER_POS = 78.1;
    private final double MID_CONE_ELBOW_POS = 86.6;

    private final double MID_CONE_SHOULDER_PLACE_POS = 78.1;
    private final double MID_CONE_ELBOW_PLACE_POS = 67.1;

    private final double MID_CUBE_SHOULDER_POS = 107.8;
    private final double MID_CUBE_ELBOW_POS = 80.7;

    private final double HIGH_CONE_SPIT_SHOULDER_POS = 40.2;
    private final double HIGH_CONE_SPIT_ELBOW_POS = 105.9;

    // this is set to be the height prior to dropping. ignore the name. 
    // needs refactored
    private final double HIGH_CONE_DROP_SHOULDER_POS = 39.7;
    private final double HIGH_CONE_DROP_ELBOW_POS = 118.5;

    private final double HIGH_CUBE_SHOULDER_POS = 77.3;
    private final double HIGH_CUBE_ELBOW_POS = 95.0;

    private final double HUMAN_PLAYER_SHOULDER_POS = 113.5;
    private final double HUMAN_PLAYER_ELBOW_POS = 92.5;

    private final double GROUND_PICKUP_SHOULDER_POS = 46.2;
    private final double GROUND_PICKUP_ELBOW_POS = -1.14;

    private final double INTERMEDIATE_SCORING_SHOULDER_POS = 95;
    private final double INTERMEDIATE_SCORING_ELBOW_POS = 31;

    private final double INTERMEDIATE_PICKUP_SHOULDER_POS = 50;
    private final double INTERMEDIATE_PICKUP_ELBOW_POS = 25;

    private final double INTERMEDIATE_ALL_SHOULDER_POS = 109.6;
    private final double INTERMEDIATE_ALL_ELBOW_POS = 25.7;

    /**
     * constants for the above defined in the ArmPosition Enum.
     * This is the place to make edits to setpoints.
     */
    private final double STORED_Y_POS = 15.75;
    private final double STORED_X_POS = -9;
    private final WristPosition STORED_WRIST_POS = WristPosition.Perpendicular;
    private final Arm2DPosition STORED_SETPOINT = new Arm2DPosition(STORED_Y_POS,
            STORED_X_POS,
            STORED_WRIST_POS);

    private final double GROUND_PICKUP_Y_POS = 0.0;
    private final double GROUND_PICKUP_X_POS = 0.0;
    private final WristPosition GROUND_PICKUP_WRIST_POS = WristPosition.Perpendicular;
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
    private final double INTAKE_HAS_PIECE_CURRENT = 32;

    private Solenoid _wristSolenoid;

    /** Arm elbow and wrist potentionmeters. Measurers arm angle */
    private AnalogInput _elbowPotentiometer;
    private AnalogInput _shoulderPotentiometer;

    // Arm PID controllers
    private ProfiledPIDController _shoulderMotorPID;
    private ProfiledPIDController _elbowMotorPID;

    private ArmPosition _currentArmPos;
    private ArmMode _armMode;

    // shoulder PID constants
    private final double SHOULDER_MOTOR_KP = 0.015;
    private final double SHOULDER_MOTOR_KI = 0.002;
    private final double SHOULDER_MOTOR_KD = 0.0;
    private final double SHOULDER_MOTOR_TOLERANCE = 1.0;

    // shoulder motion profile constraints
    private final double SHOULDER_MAX_VELOCITY = 70; // max speed that this joint should move at
    private final double SHOULDER_MAX_ACCELERATION = 40; // max acceleration this joint should move at
    private final TrapezoidProfile.Constraints SHOULDER_MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(
            SHOULDER_MAX_VELOCITY,
            SHOULDER_MAX_ACCELERATION);

    // Elbow constants
    private final double ELBOW_MOTOR_KP = 0.0175;
    private final double ELBOW_MOTOR_KI = 0.001;
    private final double ELBOW_MOTOR_KD = 0.00;
    private final double ELBOW_MOTOR_TOLERANCE = 2.0;

    // shoulder motion profile constraints
    private final double ELBOW_MAX_VELOCITY = 70; // max speed that this joint should move at
    private final double ELBOW_MAX_ACCELERATION = 60; // max acceleration this joint should move at
    private final TrapezoidProfile.Constraints ELBOW_MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(
            ELBOW_MAX_VELOCITY,
            ELBOW_MAX_ACCELERATION);

    public Arm() {
        // Motor Controllers and pnuematics
        _shoulderMotor = new WPI_TalonFX(Constants.CanIDs.ARM_SHOULDER_MOTOR_ID);
        _elbowMotor = new WPI_TalonFX(Constants.CanIDs.ARM_ELBOW_MOTOR_ID);

        _wristSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.PHPorts.WRIST_SOLENOID_PORT);

        _elbowPotentiometer = new AnalogInput(Constants.AIO.ELBOW_PORT_POT);
        _shoulderPotentiometer = new AnalogInput(Constants.AIO.SHOULDER_PORT_POT);
        _elbowPotentiometer.setAverageBits(2);
        _elbowPotentiometer.setOversampleBits(0);
        _shoulderPotentiometer.setAverageBits(2);
        _shoulderPotentiometer.setOversampleBits(0);

        // Set the motors to adjust their output based on battery voltage
        _shoulderMotor.configVoltageCompSaturation(this.MAX_MOTOR_VOLTAGE);
        _shoulderMotor.enableVoltageCompensation(true);
        _elbowMotor.configVoltageCompSaturation(this.MAX_MOTOR_VOLTAGE);
        _elbowMotor.enableVoltageCompensation(true);

        // TODO: current limit is 0.83 amps
        // _shoulderMotor.configStatorCurrentLimit(new
        // StatorCurrentLimitConfiguration())

        // set motor breaking
        _shoulderMotor.setNeutralMode(NeutralMode.Brake);
        _elbowMotor.setNeutralMode(NeutralMode.Brake);

        // Set inversion
        _shoulderMotor.setInverted(InvertType.None);
        _elbowMotor.setInverted(InvertType.None);

        // Intake
        _intakeMotor = new CANSparkMax(Constants.CanIDs.ARM_INTAKE_MOTOR_ID, MotorType.kBrushless);
        _intakeMotor.setInverted(true);
        _intakeMotor.setIdleMode(IdleMode.kBrake);

        _currentArmPos = ArmPosition.Stored;
        _armMode = ArmMode.Cube;

        // Create Shoulder PID controller
        _shoulderMotorPID = new ProfiledPIDController(this.SHOULDER_MOTOR_KP,
                this.SHOULDER_MOTOR_KI,
                this.SHOULDER_MOTOR_KD,
                this.SHOULDER_MOTION_PROFILE_CONSTRAINTS);
        _shoulderMotorPID.setTolerance(this.SHOULDER_MOTOR_TOLERANCE);

        // create elbow PID controller
        _elbowMotorPID = new ProfiledPIDController(this.ELBOW_MOTOR_KP,
                this.ELBOW_MOTOR_KI,
                this.ELBOW_MOTOR_KD,
                this.ELBOW_MOTION_PROFILE_CONSTRAINTS);
        _elbowMotorPID.setTolerance(this.ELBOW_MOTOR_TOLERANCE);

        this.updateShuffleBoard();
    }

    private void updateShuffleBoard() {
        SmartDashboard.putNumber("Elbow Angle", this.getElbowJointAngle());
        SmartDashboard.putNumber("Shoulder Angle", this.getShoulderJointAngle());
        SmartDashboard.putNumber("Sholder Angle Setpoint", this.getShoulderSetpoint());
        SmartDashboard.putNumber("Elbow Angle Setpoint", this.getElbowSetpoint());
        SmartDashboard.putNumber("Elbow Pot", this.getElbowPotPos());
        SmartDashboard.putNumber("Shoulder Pot", this.getShoulderPotPos());
        SmartDashboard.putNumber("Elbow Motor Output", this._elbowMotor.get());
        SmartDashboard.putNumber("Shoulder Motor Output", this._shoulderMotor.get());
        SmartDashboard.putString("Current Position", this._currentArmPos.toString());
        SmartDashboard.putString("Current Mode", this._armMode.toString());
    }

    private void shuffleBoardInit() {
        ShuffleboardTab tab = Shuffleboard.getTab("Arm");
    }

    private double getElbowSetpoint() {
        return this._elbowMotorPID.getSetpoint().position;
    }

    private double getShoulderSetpoint() {
        return this._shoulderMotorPID.getSetpoint().position;
    }

    /**
     * Sets the arm position
     * 
     * @param position Position to set the arm wrist to.
     */
    public void setWristPosition(WristPosition position) {
        System.out.println("Setting wrist position to " + position);
        _wristSolenoid.set(position == WristPosition.Parallel);
    }

    /**
     * @return the current position of the wrist
     */
    public WristPosition getWristPosition() {
        return _wristSolenoid.get() ? WristPosition.Perpendicular : WristPosition.Parallel;
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
    public ArmJointAngles jointAnglesFrom2DPose(Arm2DPosition arm2DPosition) {
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
        // parenthesis on line 419. that might
        // have been the issue... :( Still moving forward with this code for now since I
        // have more confidance in it.
        // Here is the commit I refered to:
        // https://github.com/FRC5188/ChargedCode/commit/a35d8567eb73343b4be02176031213db640c6627

        // grab the points
        double z = arm2DPosition.getz() - this.SHOULDER_JOINT_Z_POS;
        double y = arm2DPosition.gety() - this.SHOULDER_JOINT_Y_POS;
        double elbowLen = this.ELBOW_ARM_LENGTH;
        double shoulderLen = this.SHOULDER_ARM_LENGTH;

        // alpha = acos((x^2 + y^2 - a1^2 - a2^2) / (-2*a1*a2))
        double alpha = Math.acos((Math.pow(y, 2) + Math.pow(z, 2) -
                Math.pow(elbowLen, 2) - Math.pow(shoulderLen, 2)) /
                (-2 * elbowLen * shoulderLen));

        // this is a temp variable. following my math sheet
        double t1 = Math.atan(z / y);
        // this is also a temp variable. following my math sheet
        double gamma = Math.PI - alpha;

        // t2 = atan(a2*sin(q2))/(a1 + a2*cos(q2))
        double t2 = Math.atan((elbowLen * Math.sin(gamma)) /
                (shoulderLen + elbowLen * Math.cos(gamma)));

        double shoulderAngle = t1 + t2;
        double elbowAngle = (-1 * Math.PI + shoulderAngle + alpha);

        return new ArmJointAngles(Math.toDegrees(shoulderAngle), Math.toDegrees(elbowAngle));
    }

    /**
     * Get the current position of the arm in 2d space and return in
     * 
     * @return a Arm2DPosition that represents the wrist position in 2D space
     */
    public Arm2DPosition getArm2DPosition() {
        double currentShoulder = this.getShoulderJointAngle();
        double currentElbow = this.getElbowJointAngle();
        WristPosition wristPos = this.getWristPosition();
        Arm2DPosition currentPos = this.arm2DPositionFromAngles(currentShoulder, currentElbow, wristPos);

        return currentPos;
    }

    /**
     * Convert the current pot value of the eblow into an angle.
     * 
     * @return Current elbow joint angle relative to the shoulder
     */
    public double getElbowJointAngle() {
        double diff = this.ELBOW_neg90_DEGREE_POT_OFFSET - this.ELBOW_0_DEGREE_POT_OFFSET;
        double potValPerDegree = diff / 90;
        double curAnglePot = this.getElbowPotPos() - this.ELBOW_0_DEGREE_POT_OFFSET;
        return -curAnglePot / potValPerDegree + this.getShoulderJointAngle() + 90; // returns degrees
    }

    /**
     * Convert the current pot value of the shoulder into an angle.
     * 
     * @return Current shoulder joint angle where 90 is perpendicular to the ground
     */
    public double getShoulderJointAngle() {
        double diff = this.SHOULDER_90_DEGREE_POT_OFFSET - this.SHOULDER_0_DEGREE_POT_OFFSET;
        double potValPerDegree = diff / 90;
        double curAnglePot = this.getShoulderPotPos() - this.SHOULDER_0_DEGREE_POT_OFFSET;
        return curAnglePot / potValPerDegree; // returns degrees
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
        // TODO check this math

        double y = this.SHOULDER_JOINT_Y_POS;
        y += this.SHOULDER_ARM_LENGTH * Math.cos(currentShoulder);
        y += this.ELBOW_ARM_LENGTH * Math.cos(currentElbow);

        double z = this.SHOULDER_JOINT_Z_POS;
        z += this.SHOULDER_ARM_LENGTH * Math.sin(currentShoulder);
        z += this.ELBOW_ARM_LENGTH * Math.sin(currentElbow);

        return new Arm2DPosition(y, z, wristPos);
    }

    /**
     * Executes the shoulder motor PID controller. This PID controller will handle
     * ramping the PID output and setting any limits.
     */
    public void shoulderMotorPIDExec() {
        double angle = getShoulderJointAngle();
        double speed = _shoulderMotorPID.calculate(angle);
        setShoulderMotorSpeed(speed);
    }

    /**
     * Executes the elbow motor PID controller. This PID controller will handle
     * ramping the PID output and setting any limits.
     */
    public void elbowMotorPIDExec() {
        double angle = getElbowJointAngle();
        double speed = _elbowMotorPID.calculate(angle);
        setElbowMotorSpeed(speed);
    }

    /**
     * Finds the error, which is how far away from our shoulder setpoint we are
     * @return The error of the shoulder
     */
    public double getShoulderError() {
        return Math.abs(_shoulderMotorPID.getGoal().position - this.getShoulderJointAngle());
    }

    /**
     * Finds the error, which is how far away from our elbow setpoint we are
     * @return The error of the elbow
     */
    public double getElbowError() {
        return Math.abs(_elbowMotorPID.getGoal().position - this.getElbowJointAngle());
    }

    /** Checks if the joint motors are at their setpoints **/
    public boolean shoulderAtSetpoint() {
        return getShoulderError() < SHOULDER_MOTOR_TOLERANCE;
    }

    /** Checks if the joint motors are at their setpoints **/
    public boolean elbowAtSetpoint() {
        return getElbowError() < ELBOW_MOTOR_TOLERANCE;
    }

    public void setShoulderMotorSpeed(double speed) {

        if (this.getShoulderJointAngle() <= this.SHOULDER_LOWER_SOFT_STOP && speed < 0) {
            speed = 0;
        } else if (this.getShoulderJointAngle() >= this.SHOULDER_UPPER_SOFT_STOP && speed > 0) {
            speed = 0;
        }
        SmartDashboard.putNumber("Shoulder speed", speed);
        _shoulderMotor.set(speed);
    }

    public void setElbowMotorSpeed(double speed) {
        if (this.getElbowJointAngle() <= this.ELBOW_LOWER_SOFT_STOP && speed < 0) {
            speed = 0;
        } else if (this.getElbowJointAngle() >= this.ELBOW_UPPER_SOFT_STOP && speed > 0) {
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

    /**
     * Checks if we have a gamepiece based on current spikes.
     * When the motor starts stalling, which it does when we have a piece,
     * the current will spike. So we check if we have spiked, meaning we
     * have a piece, and send back a true when the motor has started stalling
     * @return true if we have a gamepiece based on stalling current, false otherwise
     */
    public boolean intakeHasPiece() {
        return _intakeMotor.getOutputCurrent() >= INTAKE_HAS_PIECE_CURRENT;
    }

    public void setWristPosition(ArmPosition armPosition) {
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
                setWristPosition(STORED_WRIST_POS);
                break;
            case Middle:
                if (_armMode == ArmMode.Cone) {
                    setWristPosition(MIDDLE_CONE_WRIST_POS);
                } else {
                    setWristPosition(MIDDLE_CUBE_WRIST_POS);
                }
                break;
            case High:
                if (_armMode == ArmMode.Cone) {
                    setWristPosition(HIGH_CONE_WRIST_POS);
                } else {
                    setWristPosition(HIGH_CUBE_WRIST_POS);
                }
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

    public ArmPosition getCurrentArmPosition() {
        return _currentArmPos;
    }

    public void setArmGoalsFromPosition(Arm2DPosition position) {
        ArmJointAngles goalAngles = this.jointAnglesFrom2DPose(position);
        _shoulderMotorPID.reset(this.getShoulderJointAngle());
        _shoulderMotorPID.setGoal(goalAngles.getShoulderJointAngle());
        _elbowMotorPID.reset(this.getElbowJointAngle());
        _elbowMotorPID.setGoal(goalAngles.getElbowJointAngle());
    }

    public void setArmGoalsFromPosition(ArmPosition position) {
        double shoulderPos = this.getShoulderJointAngle();
        double elbowPos = this.getElbowJointAngle();
        _currentArmPos = position;

        switch (position) {
            case GroundPickUp:
                shoulderPos = GROUND_PICKUP_SHOULDER_POS;
                elbowPos = GROUND_PICKUP_ELBOW_POS;
                break;
            case HighCone:
                shoulderPos = HIGH_CONE_DROP_SHOULDER_POS;
                elbowPos = HIGH_CONE_DROP_ELBOW_POS;
                break;
            case HighCube:
                shoulderPos = HIGH_CUBE_SHOULDER_POS;
                elbowPos = HIGH_CUBE_ELBOW_POS;
                break;
            case LoadStationPickUp:
                shoulderPos = HUMAN_PLAYER_SHOULDER_POS;
                elbowPos = HUMAN_PLAYER_ELBOW_POS;
                break;
            case LowScore:
                shoulderPos = GROUND_PICKUP_SHOULDER_POS;
                elbowPos = GROUND_PICKUP_ELBOW_POS;
                break;
            case MiddleCone:
                shoulderPos = MID_CONE_SHOULDER_POS;
                elbowPos = MID_CONE_ELBOW_POS;
                break;
            case MiddleCube:
                shoulderPos = MID_CUBE_SHOULDER_POS;
                elbowPos = MID_CUBE_ELBOW_POS;
                break;
            case Stored:
                shoulderPos = STORED_SHOULDER_POS;
                elbowPos = STORED_ELBOW_POS;
                break;
            case IntermediateScoring:
                shoulderPos = INTERMEDIATE_SCORING_SHOULDER_POS;
                elbowPos = INTERMEDIATE_SCORING_ELBOW_POS;
                break;
            case IntermediatePickup:
                shoulderPos = INTERMEDIATE_PICKUP_SHOULDER_POS;
                elbowPos = INTERMEDIATE_PICKUP_ELBOW_POS;
                break;
            case Middle:
                if (_armMode == ArmMode.Cone) {
                    shoulderPos = MID_CONE_SHOULDER_POS;
                    elbowPos = MID_CONE_ELBOW_POS;
                } else {
                    shoulderPos = MID_CUBE_SHOULDER_POS;
                    elbowPos = MID_CUBE_ELBOW_POS;
                }
                break;
            case High:
                if (_armMode == ArmMode.Cone) {
                    shoulderPos = HIGH_CONE_DROP_SHOULDER_POS;
                    elbowPos = HIGH_CONE_DROP_ELBOW_POS;
                } else {
                    shoulderPos = HIGH_CUBE_SHOULDER_POS;
                    elbowPos = HIGH_CUBE_ELBOW_POS;
                }
                break;
            default:
                // If we hit default that means we don't know what position we are in
                // So we just wanna stay put until we get a new position
                break;
        }

        // Set the PIDs to their new positions
        // We first reset the PIDs so when they draw the new profile it 
        // starts from where the arm currently is, and then we give
        // the PIDs the new angles we want to go to
        _shoulderMotorPID.reset(this.getShoulderJointAngle());
        _shoulderMotorPID.setGoal(shoulderPos);
        _elbowMotorPID.reset(this.getElbowJointAngle());
        _elbowMotorPID.setGoal(elbowPos);
    }
    
    public Arm2DPosition getArm2DPoseFromPosition(ArmPosition position) {
        double shoulderPos;
        double elbowPos;
        WristPosition wristPos;
        switch (position) {
            case GroundPickUp:
                shoulderPos = GROUND_PICKUP_SHOULDER_POS;
                elbowPos = GROUND_PICKUP_ELBOW_POS;
                wristPos = GROUND_PICKUP_WRIST_POS;
                break;
            case HighCone:
                shoulderPos = HIGH_CONE_DROP_SHOULDER_POS;
                elbowPos = HIGH_CONE_DROP_ELBOW_POS;
                wristPos = HIGH_CONE_WRIST_POS;
                break;
            case HighCube:
                shoulderPos = HIGH_CUBE_SHOULDER_POS;
                elbowPos = HIGH_CUBE_ELBOW_POS;
                wristPos = HIGH_CUBE_WRIST_POS;
                break;
            case LoadStationPickUp:
                shoulderPos = HUMAN_PLAYER_SHOULDER_POS;
                elbowPos = HUMAN_PLAYER_ELBOW_POS;
                wristPos = LOAD_STATION_PICKUP_WRIST_POS;
                break;
            case LowScore:
                shoulderPos = GROUND_PICKUP_SHOULDER_POS;
                elbowPos = GROUND_PICKUP_ELBOW_POS;
                wristPos = LOW_SCORE_WRIST_POS;
                break;
            case MiddleCone:
                shoulderPos = MID_CONE_SHOULDER_POS;
                elbowPos = MID_CONE_ELBOW_POS;
                wristPos = MIDDLE_CONE_WRIST_POS;
                break;
            case MiddleCube:
                shoulderPos = MID_CUBE_SHOULDER_POS;
                elbowPos = MID_CUBE_ELBOW_POS;
                wristPos = MIDDLE_CUBE_WRIST_POS;
                break;
            case Stored:
                shoulderPos = STORED_SHOULDER_POS;
                elbowPos = STORED_ELBOW_POS;
                wristPos = STORED_WRIST_POS;
                break;
            case IntermediateScoring:
                shoulderPos = INTERMEDIATE_SCORING_SHOULDER_POS;
                elbowPos = INTERMEDIATE_SCORING_ELBOW_POS;
                wristPos = getWristPosition();
                break;
            case IntermediatePickup:
                shoulderPos = INTERMEDIATE_PICKUP_SHOULDER_POS;
                elbowPos = INTERMEDIATE_PICKUP_ELBOW_POS;
                wristPos = getWristPosition();
                break;
            case Middle:
                if (_armMode == ArmMode.Cone) {
                    shoulderPos = MID_CONE_SHOULDER_POS;
                    elbowPos = MID_CONE_ELBOW_POS;
                    wristPos = MIDDLE_CONE_WRIST_POS;
                } else {
                    shoulderPos = MID_CUBE_SHOULDER_POS;
                    elbowPos = MID_CUBE_ELBOW_POS;
                    wristPos = MIDDLE_CUBE_WRIST_POS;
                }
                break;
            case High:
                if (_armMode == ArmMode.Cone) {
                    shoulderPos = HIGH_CONE_DROP_SHOULDER_POS;
                    elbowPos = HIGH_CONE_DROP_ELBOW_POS;
                    wristPos = HIGH_CONE_WRIST_POS;
                } else {
                    shoulderPos = HIGH_CUBE_SHOULDER_POS;
                    elbowPos = HIGH_CUBE_ELBOW_POS;
                    wristPos = HIGH_CUBE_WRIST_POS;
                }
                break;
            default:
                shoulderPos = getShoulderJointAngle();
                elbowPos = getElbowJointAngle();
                wristPos = getWristPosition();
                // If we hit default that means we don't know what position we are in
                // So we just wanna stay put until we get a new position
                break;
        }
        return arm2DPositionFromAngles(shoulderPos, elbowPos, wristPos);
    }

    /**
     * Finds what intermediate position to use, if any, for the position we want the 
     * arm to move to
     * @param position The desired final position
     * @return The intermediate position the arm must move to before the final position
     */
    public ArmPosition getIntermediatePosition(ArmPosition position) {
        ArmPosition interPos = position;

        // We only want to run these intermediate positions if we are going somewhere
        // from stow
        if (_currentArmPos == ArmPosition.Stored) {
            switch (position) {
                case LowScore:
                case GroundPickUp:
                    interPos = ArmPosition.IntermediatePickup;
                    break;
                case Stored:
                    break;
                default:
                    interPos = ArmPosition.IntermediateScoring;
                    break;
            }
        }

        // We only want to run these intermediate positions if we are going somewhere
        // from ground pickup
        if (_currentArmPos == ArmPosition.GroundPickUp || _currentArmPos == ArmPosition.LowScore) {
            switch (position) {
                case LowScore:
                case GroundPickUp:
                    break;
                default:
                    interPos = ArmPosition.IntermediatePickup;
                    break;
            }
        }

        return interPos;
    }

    public ArmMode getArmMode() {
        return _armMode;
    }

    public void setArmMode(ArmMode mode) {
        _armMode = mode;
    }

    @Override
    public void periodic() {
        // Update the dashboard
        this.updateShuffleBoard();
    }
}
