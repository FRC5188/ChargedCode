// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.arm.ArmConstants.TrajectorySpeeds;

public class Arm extends SubsystemBase {

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
        TippedConePickUp,
        HighCube,
        HighCone,
        LoadStationPickUp,
        LowScore,
        MiddleCone,
        MiddleCube,
        IntermediateScoring,
        EnGarde,
        IntermediateToPickup,
        IntermediateFromPickup,
        Middle,
        High,
        ScoringConeMiddle,
        ScoringConeHigh
    }

    public enum ArmMode {
        Cone,
        Cube
    }

    /* Arm Motor Controllers and Pnuematics. */
    private WPI_TalonFX _shoulderMotor;
    private WPI_TalonFX _elbowMotor;

    private CANSparkMax _intakeMotor;

    private Solenoid _wristSolenoid;
    private DoubleSolenoid _intakeSolenoid;

    // Helps keep track of where we are at and where we want to go
    private ArmPosition _currentArmPos;
    private ArmPosition _targetPosition;
    private ArmPosition _finalPosition;

    /** Arm elbow and wrist potentionmeters. Measures arm angle **/
    private AnalogInput _elbowPotentiometer;
    private AnalogInput _shoulderPotentiometer;

    // Arm PID controllers
    private PIDController _shoulderMotorPID;
    private PIDController _elbowMotorPID;

    private ArmMode _armMode;

    private boolean _hasGamepiece = true;
    private IntakeMode _intakeMode;

    // If true, we can change where the arm is moving to
    private boolean _canChangeSetpoint;

    // Set this to true so that the arm is in coast and the motors don't run
    // WARNING: DOESN'T UPDATE THE POTS, WILL ALWAYS ASSUME THAT THE ROBOT IS IN ONE
    // POSITION
    // setCurrentPosition isn't updated
    private boolean _inSetpointTestingMode = false;

    private boolean _pidEnable;

    // Stuff to run arm trajectories
    private ArmTrajectory _trajectory;
    private boolean _runningTrajectory = false;
    private Timer _trajTimer = new Timer();

    // constructor
    public Arm() {
        // Motor Controllers and pnuematics
        _shoulderMotor = new WPI_TalonFX(Constants.CanIDs.ARM_SHOULDER_MOTOR_ID);
        _elbowMotor = new WPI_TalonFX(Constants.CanIDs.ARM_ELBOW_MOTOR_ID);

        _wristSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.PHPorts.WRIST_SOLENOID_PORT);
        _intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PHPorts.INTAKE_SOLENOID_FORWARD_PORT,
                Constants.PHPorts.INTAKE_SOLENOID_REVERSE_PORT);

        _elbowPotentiometer = new AnalogInput(Constants.AIO.ELBOW_PORT_POT);
        _shoulderPotentiometer = new AnalogInput(Constants.AIO.SHOULDER_PORT_POT);
        _elbowPotentiometer.setAverageBits(2);
        _elbowPotentiometer.setOversampleBits(0);
        _shoulderPotentiometer.setAverageBits(2);
        _shoulderPotentiometer.setOversampleBits(0);

        // Set the motors to adjust their output based on battery voltage
        _shoulderMotor.configVoltageCompSaturation(ArmConstants.MAX_MOTOR_VOLTAGE);
        _shoulderMotor.enableVoltageCompensation(true);
        _elbowMotor.configVoltageCompSaturation(ArmConstants.MAX_MOTOR_VOLTAGE);
        _elbowMotor.enableVoltageCompensation(true);

        // set motor breaking
        _shoulderMotor.setNeutralMode((_inSetpointTestingMode) ? NeutralMode.Coast : NeutralMode.Brake);
        _elbowMotor.setNeutralMode((_inSetpointTestingMode) ? NeutralMode.Coast : NeutralMode.Brake);

        // Set inversion
        _shoulderMotor.setInverted(InvertType.None);
        _elbowMotor.setInverted(InvertType.None);

        // Intake
        _intakeMotor = new CANSparkMax(Constants.CanIDs.ARM_INTAKE_MOTOR_ID, MotorType.kBrushless);
        _intakeMotor.setInverted(true);
        _intakeMotor.setIdleMode(IdleMode.kBrake);

        _targetPosition = ArmPosition.Stored;
        _finalPosition = ArmPosition.Stored;
        _currentArmPos = ArmPosition.Stored;
        _armMode = ArmMode.Cube;
        _intakeMode = IntakeMode.Closed;

        // Create Shoulder PID controller
        _shoulderMotorPID = new PIDController(ArmConstants.SHOULDER_MOTOR_KP,
                ArmConstants.SHOULDER_MOTOR_KI,
                ArmConstants.SHOULDER_MOTOR_KD);
        _shoulderMotorPID.setTolerance(ArmConstants.SHOULDER_MOTOR_TOLERANCE);

        // create elbow PID controller
        _elbowMotorPID = new PIDController(ArmConstants.ELBOW_MOTOR_KP,
                ArmConstants.ELBOW_MOTOR_KI,
                ArmConstants.ELBOW_MOTOR_KD);
        _elbowMotorPID.setTolerance(ArmConstants.ELBOW_MOTOR_TOLERANCE);

        _canChangeSetpoint = true;

        this.enablePID();
        this.updateShuffleBoard();
    }

    public void setTestMode(boolean inTest) {
        // set motor breaking
        _shoulderMotor.setNeutralMode((inTest) ? NeutralMode.Coast : NeutralMode.Brake);
        _elbowMotor.setNeutralMode((inTest) ? NeutralMode.Coast : NeutralMode.Brake);
        _inSetpointTestingMode = inTest;
    }

    public boolean canChangeSetpoint() {
        return _canChangeSetpoint;
    }

    public void setCanChangeSetpoint(boolean canChange) {
        SmartDashboard.putBoolean("Can Change Setpoint", canChange);
        _canChangeSetpoint = canChange;
    }

    public ArmPosition getTargetArmPosition() {
        return this._targetPosition;
    }

    public ArmPosition getFinalPosition() {
        return this._finalPosition;
    }

    public void setFinalPosition(ArmPosition position) {
        this._finalPosition = position;
    }

    public boolean atFinalPosition() {
        ArmJointAngles angles = getArmAnglesFromPosition(_targetPosition);
        return (Math.abs(angles.shoulderJointAngle - getShoulderJointAngle()) < ArmConstants.SHOULDER_MOTOR_TOLERANCE)
                &&
                (Math.abs(angles.elbowJointAngle
                        - getElbowJointAngleRelativeToGround()) < ArmConstants.ELBOW_MOTOR_TOLERANCE);
    }

    public void setCurrentPosition(ArmPosition inputArmPosition) {
        // System.out.println("SETTING POSITION __________" + inputArmPosition);
        this._currentArmPos = inputArmPosition;
    }

    private void updateShuffleBoard() {
        SmartDashboard.putNumber("Elbow Angle Relative to Ground", this.getElbowJointAngleRelativeToGround());
        SmartDashboard.putNumber("Elbow Angle Relative to Shoulder", this.getElbowJointAngleRelativeToShoulder());
        SmartDashboard.putNumber("Shoulder Angle", this.getShoulderJointAngle());
        SmartDashboard.putNumber("Sholder Angle Setpoint", this.getShoulderSetpoint());
        SmartDashboard.putNumber("Elbow Angle Setpoint", this.getElbowSetpoint());
        SmartDashboard.putNumber("Elbow Pot", this.getElbowPotPos());
        SmartDashboard.putNumber("Elbow Motor Output", this._elbowMotor.get());
        SmartDashboard.putNumber("Shoulder Motor Output", this._shoulderMotor.get());
        SmartDashboard.putString("Current Position", this._currentArmPos.toString());
        SmartDashboard.putString("Current Mode", this._armMode.toString());
        SmartDashboard.putString("Target Position", this._targetPosition.toString());
        SmartDashboard.putString("Final Scoring Position", this._finalPosition.toString());
    }

    public double getElbowSetpoint() {
        // return this._elbowMotorPID.getSetpoint().position;
        return this._elbowMotorPID.getSetpoint();
    }

    public double getShoulderSetpoint() {
        // return this._shoulderMotorPID.getSetpoint().position;
        return this._shoulderMotorPID.getSetpoint();
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
        return _wristSolenoid.get() ? WristPosition.Parallel : WristPosition.Perpendicular;
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
        double y = arm2DPosition.gety() - ArmConstants.SHOULDER_JOINT_Y_POS;
        double z = arm2DPosition.getz() - ArmConstants.SHOULDER_JOINT_Z_POS;
        double elbowLen = ArmConstants.ELBOW_LENGTH;
        double shoulderLen = ArmConstants.SHOULDER_LENGTH;

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
     * Convert the current pot value of the eblow into an angle.
     * 
     * @return Current elbow joint angle relative to the ground
     */
    public double getElbowJointAngleRelativeToGround() {
        double diff = ArmConstants.ELBOW_0_DEGREE_POT_OFFSET - ArmConstants.ELBOW_90_DEGREE_POT_OFFSET;
        double potValPerDegree = diff / 90;
        double curAnglePot = this.getElbowPotPos() - ArmConstants.ELBOW_90_DEGREE_POT_OFFSET;
        return -curAnglePot / potValPerDegree + this.getShoulderJointAngle() + 90; // returns degrees
    }

    /**
     * Convert the current pot value of the eblow into an angle.
     * 
     * @return Current elbow joint angle relative to the ground
     */
    public double getElbowJointAngleRelativeToShoulder() {
        double diff = ArmConstants.ELBOW_0_DEGREE_POT_OFFSET - ArmConstants.ELBOW_90_DEGREE_POT_OFFSET;
        double potValPerDegree = diff / 90;
        double curAnglePot = this.getElbowPotPos() - ArmConstants.ELBOW_90_DEGREE_POT_OFFSET;
        return -curAnglePot / potValPerDegree + 180; // returns degrees
    }

    /**
     * Convert the current pot value of the shoulder into an angle.
     * 
     * @return Current shoulder joint angle where 90 is perpendicular to the ground
     */
    public double getShoulderJointAngle() {
        double diff = ArmConstants.SHOULDER_90_DEGREE_POT_OFFSET - ArmConstants.SHOULDER_0_DEGREE_POT_OFFSET;
        double potValPerDegree = diff / 90;
        double curAnglePot = this.getShoulderPotPos() - ArmConstants.SHOULDER_0_DEGREE_POT_OFFSET;
        return curAnglePot / potValPerDegree; // returns degrees
    }

    public void execPIDs() {
        runTrajectory();
        shoulderMotorPIDExec();
        elbowMotorPIDExec();
    }

    /**
     * Executes the shoulder motor PID controller. This PID controller will handle
     * ramping the PID output and setting any limits.
     */
    private void shoulderMotorPIDExec() {

        double angle = getShoulderJointAngle();
        setShoulderMotorSpeed(_shoulderMotorPID.calculate(angle)
                + FeedForward.shoulder(-getElbowJointAngleRelativeToShoulder() + 180, getShoulderJointAngle()));
    }

    /**
     * Executes the elbow motor PID controller. This PID controller will handle
     * ramping the PID output and setting any limits.
     */
    private void elbowMotorPIDExec() {
        double angle = getElbowJointAngleRelativeToGround();
        setElbowMotorSpeed(_elbowMotorPID.calculate(angle)
                + FeedForward.elbow(-getElbowJointAngleRelativeToShoulder() + 180, getShoulderJointAngle()));
    }

    /**
     * Finds the error, which is how far away from our shoulder setpoint we are
     * 
     * @return The error of the shoulder
     */
    public double getShoulderError() {
        // return Math.abs(_shoulderMotorPID.getGoal().position -
        // this.getShoulderJointAngle());
        return Math.abs(_shoulderMotorPID.getSetpoint() - this.getShoulderJointAngle());
    }

    /**
     * Finds the error, which is how far away from our elbow setpoint we are
     * 
     * @return The error of the elbow
     */
    public double getElbowError() {
        // return Math.abs(_elbowMotorPID.getGoal().position -
        // this.getElbowJointAngleRelativeToGround());
        return Math.abs(_elbowMotorPID.getSetpoint() - this.getElbowJointAngleRelativeToGround());
    }

    /** Checks if the joint motors are at their setpoints **/
    public boolean shoulderAtSetpoint() {
        return getShoulderError() < ArmConstants.SHOULDER_MOTOR_TOLERANCE;
    }

    /** Checks if the joint motors are at their setpoints **/
    public boolean elbowAtSetpoint() {
        return getElbowError() < ArmConstants.ELBOW_MOTOR_TOLERANCE;
    }

    public void setShoulderMotorSpeed(double speed) {

        if (this.getShoulderJointAngle() <= ArmConstants.SHOULDER_LOWER_SOFT_STOP && speed < 0) {
            speed = 0;
        } else if (this.getShoulderJointAngle() >= ArmConstants.SHOULDER_UPPER_SOFT_STOP && speed > 0) {
            speed = 0;
        }
        SmartDashboard.putNumber("Shoulder speed", speed);
        if (!_inSetpointTestingMode)
            _shoulderMotor.set(speed);
    }

    public void setElbowMotorSpeed(double speed) {
        if (this.getElbowJointAngleRelativeToGround() <= ArmConstants.ELBOW_LOWER_SOFT_STOP && speed < 0) {
            speed = 0;
        } else if (this.getElbowJointAngleRelativeToGround() >= ArmConstants.ELBOW_UPPER_SOFT_STOP && speed > 0) {
            speed = 0;
        }
        SmartDashboard.putNumber("Elbow speed", speed);
        if (!_inSetpointTestingMode)
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
        if (speed > 0.0) {
            _hasGamepiece = false;
        }
        _intakeMotor.set(speed);
    }

    /**
     * Is an enum that represents whether or not the claw is opened or closed.
     */
    public enum IntakeMode {
        Open,
        Closed
    }

    /**
     * Sets the intake mode
     */
    public void setIntakeMode(IntakeMode mode) {
        this._intakeMode = mode;
        _intakeSolenoid.set((_intakeMode == IntakeMode.Closed) ? Value.kForward : Value.kReverse);
    }

    /**
     * Returns the intake mode
     */
    public IntakeMode getIntakeMode() {
        return _intakeMode;
    }

    /**
     * Checks if we have a gamepiece based on current spikes.
     * When the motor starts stalling, which it does when we have a piece,
     * the current will spike. So we check if we have spiked, meaning we
     * have a piece, and send back a true when the motor has started stalling
     * 
     * @return true if we have a gamepiece based on stalling current, false
     *         otherwise
     */
    public boolean intakeHasPiece() {
        // _hasGamepiece = true; old logic -gh 3/24/23
        _hasGamepiece = _intakeMotor.getOutputCurrent() >= ArmConstants.INTAKE_HAS_PIECE_CURRENT;
        return _intakeMotor.getOutputCurrent() >= ArmConstants.INTAKE_HAS_PIECE_CURRENT;
    }

    public boolean checkGamepiece() {
        return _hasGamepiece;
    }

    public void setHasGamepiece(boolean hasGamepiece) {
        _hasGamepiece = hasGamepiece;
    }

    public boolean elbowIsHittingObject() {
        return (_elbowMotor.getStatorCurrent() > ArmConstants.ELBOW_IS_HITTING_CURRENT);
        // TODO: Check if .getStatorCurrent() is right

    }

    public void setWristPosition(ArmPosition armPosition) {
        // Sets Wrist Position based off of arm position
        switch (armPosition) {
            case GroundPickUp:
                setWristPosition(ArmConstants.SetPoints2D.GROUND_PICKUP_WRIST_POS);
                break;
            case TippedConePickUp:
                setWristPosition(ArmConstants.SetPoints2D.TIPPED_CONE_WRIST_POS);
                break;
            case HighCone:
                setWristPosition(ArmConstants.SetPoints2D.HIGH_CONE_WRIST_POS);
                break;
            case HighCube:
                setWristPosition(ArmConstants.SetPoints2D.HIGH_CUBE_WRIST_POS);
                break;
            case LoadStationPickUp:
                setWristPosition(ArmConstants.SetPoints2D.LOAD_STATION_PICKUP_WRIST_POS);
                break;
            case LowScore:
                setWristPosition(ArmConstants.SetPoints2D.LOW_SCORE_WRIST_POS);
                break;
            case MiddleCone:
                setWristPosition(ArmConstants.SetPoints2D.MIDDLE_CONE_WRIST_POS);
                break;
            case MiddleCube:
                setWristPosition(ArmConstants.SetPoints2D.MIDDLE_CUBE_WRIST_POS);
                break;
            case Stored:
                setWristPosition(ArmConstants.SetPoints2D.STORED_WRIST_POS);
                break;
            case IntermediateToPickup:
                setWristPosition(WristPosition.Perpendicular);
                break;
            case IntermediateFromPickup:
                setWristPosition(WristPosition.Perpendicular);
                break;
            case IntermediateScoring:
                setWristPosition(WristPosition.Perpendicular);
                break;
            case EnGarde:
                setWristPosition(ArmConstants.SetPoints2D.EN_GARDE_WRIST_POS);
                break;
            case Middle:
                if (_armMode == ArmMode.Cone) {
                    setWristPosition(ArmConstants.SetPoints2D.MIDDLE_CONE_WRIST_POS);
                } else {
                    setWristPosition(ArmConstants.SetPoints2D.MIDDLE_CUBE_WRIST_POS);
                }
                break;
            case High:
                if (_armMode == ArmMode.Cone) {
                    setWristPosition(ArmConstants.SetPoints2D.HIGH_CONE_WRIST_POS);
                } else {
                    setWristPosition(ArmConstants.SetPoints2D.HIGH_CUBE_WRIST_POS);
                }
                break;
            default:
                break;
        }
    }

    public Arm getInstance() {
        return this;
    }

    public boolean checkWristPosition(ArmPosition positionOfArm) {
        switch (positionOfArm) {
            case GroundPickUp:
                return getWristPosition() == ArmConstants.SetPoints2D.GROUND_PICKUP_WRIST_POS;
            case TippedConePickUp:
                return getWristPosition() == ArmConstants.SetPoints2D.GROUND_PICKUP_WRIST_POS;
            case HighCone:
                return getWristPosition() == ArmConstants.SetPoints2D.HIGH_CONE_WRIST_POS;
            case HighCube:
                return getWristPosition() == ArmConstants.SetPoints2D.HIGH_CUBE_WRIST_POS;
            case LoadStationPickUp:
                return getWristPosition() == ArmConstants.SetPoints2D.LOAD_STATION_PICKUP_WRIST_POS;
            case LowScore:
                return getWristPosition() == ArmConstants.SetPoints2D.LOW_SCORE_WRIST_POS;
            case MiddleCone:
                return getWristPosition() == ArmConstants.SetPoints2D.MIDDLE_CONE_WRIST_POS;
            case MiddleCube:
                return getWristPosition() == ArmConstants.SetPoints2D.MIDDLE_CUBE_WRIST_POS;
            case Stored:
                return getWristPosition() == ArmConstants.SetPoints2D.STORED_WRIST_POS;
            default:
                return false;
        }
    }

    public ArmPosition getCurrentArmPosition() {
        return _currentArmPos;
    }

    public void setElbowGoalFromAngle(double angle) {
        // _elbowMotorPID.reset(this.getElbowJointAngleRelativeToGround());
        // _elbowMotorPID.setGoal(angle);
        _elbowMotorPID.reset();
        _elbowMotorPID.setSetpoint(angle);
    }

    public void setArmGoalsFromPosition(Arm2DPosition position) {
        ArmJointAngles goalAngles = this.jointAnglesFrom2DPose(position);
        // _shoulderMotorPID.reset(this.getShoulderJointAngle());
        // _shoulderMotorPID.setGoal(goalAngles.getShoulderJointAngle());
        // _elbowMotorPID.reset(this.getElbowJointAngleRelativeToGround());
        // _elbowMotorPID.setGoal(goalAngles.getElbowJointAngle());
        _shoulderMotorPID.reset();
        _shoulderMotorPID.setSetpoint(goalAngles.getShoulderJointAngle());
        _elbowMotorPID.reset();
        _elbowMotorPID.setSetpoint(goalAngles.getElbowJointAngle());
    }

    public void setArmGoalsFromPosition(ArmPosition position) {
        this._targetPosition = position;

        ArmJointAngles angles = getArmAnglesFromPosition(position);

        // Set the PIDs to their new positions
        // We first reset the PIDs so when they draw the new profile it
        // starts from where the arm currently is, and then we give
        // the PIDs the new angles we want to go to
        // _shoulderMotorPID.reset(this.getShoulderJointAngle());
        // _shoulderMotorPID.setGoal(shoulderPos);
        // _elbowMotorPID.reset(this.getElbowJointAngleRelativeToGround());
        // _elbowMotorPID.setGoal(elbowPos);
        // System.out.println(
        // "Shoulder: " + _shoulderMotorPID.getGoal().position + " Elbow: " +
        // _elbowMotorPID.getGoal().position);

    }

    public void setArmGoalsFromAngles(ArmJointAngles angles) {
        // _shoulderMotorPID.reset(this.getShoulderJointAngle());
        // _shoulderMotorPID.setGoal(angles.shoulderJointAngle);
        // _elbowMotorPID.reset(this.getElbowJointAngleRelativeToGround());
        // _elbowMotorPID.setGoal(angles.elbowJointAngle);

        _shoulderMotorPID.reset();
        _shoulderMotorPID.setSetpoint(angles.getShoulderJointAngle());
        _elbowMotorPID.reset();
        _elbowMotorPID.setSetpoint(angles.getElbowJointAngle());

        System.out.println(
                "Shoulder: " + _shoulderMotorPID.getSetpoint() + " Elbow: " +
                        _elbowMotorPID.getSetpoint());
    }

    /**
     * Finds what intermediate position to use, if any, for the position we want the
     * arm to move to
     * 
     * @param position The desired final position
     * @return The intermediate position the arm must move to before the final
     *         position
     */
    public ArrayList<ArmJointAngles> getIntermediatePositions(ArmPosition position) {
        ArrayList<ArmJointAngles> intermediatePositions = new ArrayList<>();
        // We only want to run these intermediate positions if we are going somewhere
        // from stow
        System.out.println("Current Position: " + _currentArmPos + "\nNext Position: " + position);
        if (_currentArmPos == ArmPosition.Stored) {
            switch (position) {
                case LowScore:
                case GroundPickUp:
                    addWaypointsFrom2DArray(intermediatePositions,
                            ArmConstants.IntermediateWaypoints.STORED_TO_GROUND_PICKUP);
                    break;
                case Stored:
                    break;
                case EnGarde:
                    addWaypointsFrom2DArray(intermediatePositions,
                            ArmConstants.IntermediateWaypoints.STORED_TO_ENGARDE);
                    break;
                default:
                    addWaypointsFrom2DArray(intermediatePositions,
                            ArmConstants.IntermediateWaypoints.STORED_TO_SCORING);
                    break;
            }
        } else if (_currentArmPos == ArmPosition.GroundPickUp || _currentArmPos == ArmPosition.LowScore) {
            // We only want to run these intermediate positions if we are going somewhere
            // from ground pickup
            // System.out.println("Goofy Ground Position");
            switch (position) {
                case LowScore:
                case GroundPickUp:
                    break;
                case Stored:
                    addWaypointsFrom2DArray(intermediatePositions,
                            ArmConstants.IntermediateWaypoints.GROUND_PICKUP_TO_STORED);
                            break;
                default:

                    break;
            }
        } else if (_currentArmPos == ArmPosition.EnGarde) {
            System.out.println("ENGARDE ENGARDE ____________________");
            // We only want to run these intermediate positions if we are going somewhere
            // from ground pickup
            switch (position) {
                case Stored:
                    addWaypointsFrom2DArray(intermediatePositions,
                            ArmConstants.IntermediateWaypoints.ENGARDE_TO_STORED);
                    break;
                case High:
                    if (this.getArmMode() == ArmMode.Cone) {
                        addWaypointsFrom2DArray(intermediatePositions,
                                ArmConstants.IntermediateWaypoints.ENGARDE_TO_HIGH_CONE);
                    } else {
                        addWaypointsFrom2DArray(intermediatePositions,
                                ArmConstants.IntermediateWaypoints.ENGARDE_TO_HIGH_CUBE);
                    }
                    break;
                case Middle:
                    if (this.getArmMode() == ArmMode.Cone) {
                        addWaypointsFrom2DArray(intermediatePositions,
                                ArmConstants.IntermediateWaypoints.ENGUARD_TO_MID_CONE);
                    } else {

                    }
                    break;
                case GroundPickUp:
                    addWaypointsFrom2DArray(intermediatePositions,
                            ArmConstants.IntermediateWaypoints.ENGARDE_TO_GROUND_PICKUP);
                    break;
                default:

                    break;
            }
        }

        return intermediatePositions;
    }

    private void addWaypointsFrom2DArray(ArrayList<ArmJointAngles> intermediatePositions, double[][] waypoints) {
        for (int i = 0; i < waypoints.length; i++) {
            double[] angleSet = waypoints[i];
            intermediatePositions.add(new ArmJointAngles(angleSet[0], angleSet[1]));
        }
    }

    /**
     * ArmMode returns what gamepiece is being held.
     * It can be either a cube or a cone.
     */
    public ArmMode getArmMode() {
        return _armMode;
    }

    public void setArmMode(ArmMode mode) {
        _armMode = mode;
    }

    public ArmPosition getScoringPosition(ArmPosition position) {
        if (_armMode == ArmMode.Cone) {
            ArmPosition scorePos = position;

            switch (position) {
                case Middle:
                    scorePos = ArmPosition.ScoringConeMiddle;
                    break;
                case High:
                    scorePos = ArmPosition.ScoringConeHigh;
                    break;
                default:
                    break;
            }

            return scorePos;
        } else {
            return position;
        }

    }

    @Override
    public void periodic() {
        // Update the dashboard
        this.updateShuffleBoard();
        SmartDashboard.putString("Current Arm 2D", ArmTrajectory
                .arm2DPositionFromAngles(getShoulderJointAngle(), getElbowJointAngleRelativeToGround()).toString());
    }

    public void disablePID() {
        this._pidEnable = false;
    }

    public void enablePID() {
        this._pidEnable = true;
    }

    public boolean isPIDEnabled() {
        return this._pidEnable;
    }

    public void setShoulderGoalFromAngle(double setpoint) {
        _shoulderMotorPID.reset();
        _shoulderMotorPID.setSetpoint(setpoint);
    }

    public void generateTrajectory(ArmPosition position) {
        stopTrajectory();

        ArrayList<ArmJointAngles> intermediates = getIntermediatePositions(position);

        ArrayList<ArmJointAngles> waypoints = new ArrayList<>();
        waypoints.add(new ArmJointAngles(getShoulderJointAngle(), getElbowJointAngleRelativeToGround()));

        for (ArmJointAngles p : intermediates) {
            waypoints.add(p);
        }

        waypoints.add(getArmAnglesFromPosition(position));

        double speed = ArmConstants.MAX_TRAJECTORY_SPEED;
        // if we are going from loading station to stored go at a different speed
        if (this._currentArmPos == ArmPosition.LoadStationPickUp && position == ArmPosition.Stored) {
            System.out.println("SETTING TRAJECTORY SPEED: " + TrajectorySpeeds.HUMAN_PLAYER_TO_STORED_SPEED);
            speed = TrajectorySpeeds.HUMAN_PLAYER_TO_STORED_SPEED;
        } else if (this._currentArmPos == ArmPosition.EnGarde && position == ArmPosition.High) {
            System.out.println("SETTING TRAJECTORY SPEED: " + TrajectorySpeeds.ENGARDE_TO_HIGH_CONE_SPEED);
            speed = TrajectorySpeeds.ENGARDE_TO_HIGH_CONE_SPEED;
        } else if (this._currentArmPos == ArmPosition.EnGarde && position == ArmPosition.GroundPickUp) {
            speed = TrajectorySpeeds.ENGARDE_TO_GROUND_PICKUP_SPEED;
        }

        _trajectory = new ArmTrajectory(waypoints, speed);
    }

    public void startTrajectory() {
        if (!_runningTrajectory) {
            _runningTrajectory = true;
            _trajTimer.reset();
            _trajTimer.start();
        }
    }

    public boolean runTrajectory() {
        if (_runningTrajectory) {
            ArmJointAngles trajPose = _trajectory.sample(_trajTimer.get());
            setArmGoalsFromAngles(trajPose);

            if (_trajectory.getTotalTime() < _trajTimer.get()) {
                stopTrajectory();
            }
        }

        return _runningTrajectory;
    }

    public void stopTrajectory() {
        _runningTrajectory = false;
        _trajTimer.stop();
    }

    public ArmJointAngles getArmAnglesFromPosition(ArmPosition position) {
        double shoulderPos = this.getShoulderJointAngle();
        double elbowPos = this.getElbowJointAngleRelativeToGround();
        this._targetPosition = position;

        switch (position) {
            case GroundPickUp:
                shoulderPos = ArmConstants.AngleSetpoints.GROUND_PICKUP_SHOULDER_POS;
                elbowPos = ArmConstants.AngleSetpoints.GROUND_PICKUP_ELBOW_POS;
                break;
            case TippedConePickUp:
                shoulderPos = ArmConstants.AngleSetpoints.TIPPED_CONE_SHOULDER_POS;
                elbowPos = ArmConstants.AngleSetpoints.TIPPED_CONE_ELBOW_POS;
            case HighCone:
                shoulderPos = ArmConstants.AngleSetpoints.HIGH_CONE_DROP_SHOULDER_POS;
                elbowPos = ArmConstants.AngleSetpoints.HIGH_CONE_DROP_ELBOW_POS;
                break;
            case HighCube:
                shoulderPos = ArmConstants.AngleSetpoints.HIGH_CUBE_SHOULDER_POS;
                elbowPos = ArmConstants.AngleSetpoints.HIGH_CUBE_ELBOW_POS;
                break;
            case LoadStationPickUp:
                shoulderPos = ArmConstants.AngleSetpoints.HUMAN_PLAYER_SHOULDER_POS;
                elbowPos = ArmConstants.AngleSetpoints.HUMAN_PLAYER_ELBOW_POS;
                break;
            case LowScore:
                shoulderPos = ArmConstants.AngleSetpoints.GROUND_PICKUP_SHOULDER_POS;
                elbowPos = ArmConstants.AngleSetpoints.GROUND_PICKUP_ELBOW_POS;
                break;
            case MiddleCone:
                shoulderPos = ArmConstants.AngleSetpoints.MID_CONE_SHOULDER_POS;
                elbowPos = ArmConstants.AngleSetpoints.MID_CONE_ELBOW_POS;
                break;
            case MiddleCube:
                shoulderPos = ArmConstants.AngleSetpoints.MID_CUBE_SHOULDER_POS;
                elbowPos = ArmConstants.AngleSetpoints.MID_CUBE_ELBOW_POS;
                break;
            case Stored:
                shoulderPos = ArmConstants.AngleSetpoints.STORED_SHOULDER_POS;
                elbowPos = ArmConstants.AngleSetpoints.STORED_ELBOW_POS;
                break;
            case IntermediateScoring:
                shoulderPos = ArmConstants.AngleSetpoints.INTERMEDIATE_SCORING_SHOULDER_POS;
                elbowPos = ArmConstants.AngleSetpoints.INTERMEDIATE_SCORING_ELBOW_POS;
                break;
            case EnGarde:
                shoulderPos = ArmConstants.AngleSetpoints.EN_GARDE_SHOULDER_POS;
                elbowPos = ArmConstants.AngleSetpoints.EN_GARDE_ELBOW_POS;
                break;
            case IntermediateToPickup:
                shoulderPos = ArmConstants.AngleSetpoints.INTERMEDIATE_TO_PICKUP_SHOULDER_POS;
                elbowPos = ArmConstants.AngleSetpoints.INTERMEDIATE_TO_PICKUP_ELBOW_POS;
                break;
            case IntermediateFromPickup:
                shoulderPos = ArmConstants.AngleSetpoints.INTERMEDIATE_FROM_PICKUP_SHOULDER_POS;
                elbowPos = ArmConstants.AngleSetpoints.INTERMEDIATE_FROM_PICKUP_ELBOW_POS;
                break;
            case Middle:
                if (_armMode == ArmMode.Cone) {
                    shoulderPos = ArmConstants.AngleSetpoints.MID_CONE_SHOULDER_POS;
                    elbowPos = ArmConstants.AngleSetpoints.MID_CONE_ELBOW_POS;
                } else {
                    shoulderPos = ArmConstants.AngleSetpoints.MID_CUBE_SHOULDER_POS;
                    elbowPos = ArmConstants.AngleSetpoints.MID_CUBE_ELBOW_POS;
                }
                break;
            case High:
                if (_armMode == ArmMode.Cone) {
                    shoulderPos = ArmConstants.AngleSetpoints.HIGH_CONE_DROP_SHOULDER_POS;
                    elbowPos = ArmConstants.AngleSetpoints.HIGH_CONE_DROP_ELBOW_POS;
                } else {
                    shoulderPos = ArmConstants.AngleSetpoints.HIGH_CUBE_SHOULDER_POS;
                    elbowPos = ArmConstants.AngleSetpoints.HIGH_CUBE_ELBOW_POS;
                }
                break;
            case ScoringConeMiddle:
                shoulderPos = ArmConstants.AngleSetpoints.MID_CONE_SHOULDER_PLACE_POS;
                elbowPos = ArmConstants.AngleSetpoints.MID_CONE_ELBOW_PLACE_POS;
                break;
            case ScoringConeHigh:
                shoulderPos = ArmConstants.AngleSetpoints.HIGH_CONE_SPIT_SHOULDER_POS;
                elbowPos = ArmConstants.AngleSetpoints.HIGH_CONE_SPIT_ELBOW_POS;
            default:
                // If we hit default that means we don't know what position we are in
                // So we just wanna stay put until we get a new position
                break;
        }

        return new ArmJointAngles(shoulderPos, elbowPos);
    }
}
