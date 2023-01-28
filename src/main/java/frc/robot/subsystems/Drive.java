// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants.CanIDs;
import frc.robot.Constants.OperatorConstants.Position;
import frc.robot.sds.Mk4iSwerveModuleHelper;
import frc.robot.sds.SdsModuleConfigurations;
import frc.robot.sds.SwerveModule;


public class Drive extends SubsystemBase {
    
    /** The width of the chassis from the centers of the swerve modules */
    private static final double CHASSIS_WIDTH_METERS = Units.inchesToMeters(20.75);
    /** The height of the chassis from the centers of the swerve modules */
    private static final double CHASSIS_HEIGHT_METERS = Units.inchesToMeters(24.75);
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed.
     */
    public static final double MAX_VOLTAGE = 6.0;

    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     * The formula for calculating the theoretical maximum velocity is:
     * <p>
     * Motor free speed RPM / 60 * Drive reduction * Wheel diameter meters * Pi
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
            SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(CHASSIS_WIDTH_METERS / 2.0, CHASSIS_HEIGHT_METERS / 2.0);

    /** The offset to get the encoder to read 0 when facing forward */
    private static final double FRONT_LEFT_MODULE_ENCODER_OFFSET = -155.9;
    /** The offset to get the encoder to read 0 when facing forward */
    private static final double FRONT_RIGHT_MODULE_ENCODER_OFFSET = -241.17;
    /** The offset to get the encoder to read 0 when facing forward */
    private static final double BACK_LEFT_MODULE_ENCODER_OFFSET = -266.66;
    /** The offset to get the encoder to read 0 when facing forward */
    private static final double BACK_RIGHT_MODULE_ENCODER_OFFSET = -25.40;

    public static final double ksVolts = 0.166;
    public static final double kvVoltSecondsPerMeter = 2.237;
    public static final double kaVoltSecondsSquaredPerMeter = 0.389;
    public static final double kPDriveVel = 3.0166;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7; 

    /**
     * This object does the math to convert a motion vector into individual module
     * vectors
     * <p>
     * See WPILib's documentation for more information
     */
    // We divide our chassis width and height by 2 so we can represent each module
    // as a point relative to the
    // center of the robot. Positive along x means going to the left from the
    // center, and positive along
    // y means going up from the center
    private final SwerveDriveKinematics _kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(CHASSIS_WIDTH_METERS / 2.0, CHASSIS_HEIGHT_METERS / 2.0),
            // Front right
            new Translation2d(CHASSIS_WIDTH_METERS / 2.0, -CHASSIS_HEIGHT_METERS / 2.0),
            // Back left
            new Translation2d(-CHASSIS_WIDTH_METERS / 2.0, CHASSIS_HEIGHT_METERS / 2.0),
            // Back right
            new Translation2d(-CHASSIS_WIDTH_METERS / 2.0, -CHASSIS_HEIGHT_METERS / 2.0));

    /**
     * The gyro that we will use to keep track of our current rotation. The output
     * of the gyro
     * impacts the odometry of the robot, as well as field-oriented drive.
     */
    private final AHRS _navx = new AHRS();

    private SwerveDrivePoseEstimator _odometry;

    // These are our modules
    private final SwerveModule _frontLeftModule;
    private final SwerveModule _frontRightModule;
    private final SwerveModule _backLeftModule;
    private final SwerveModule _backRightModule;

    /**
     * This represents the desired vector of the robot.
     * <p>
     * The first part is the forwards velocity, which is how fast we want to go
     * forward/backward.
     * Second is the sideways velocity, which is how fast we want to strafe
     * left/right.
     * Last is the angular velocity, which is how fast we want to rotate cw/ccw.
     */
    private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private final Vision _visionSubsystem;
    /**
     * Represents the drive chassis of the robot. Contains all of the code to
     * move in a swerve format using either a joystick or supplied values.
     */
    public Drive(Vision visionSubsystem) {
        _visionSubsystem = visionSubsystem;

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        _frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            CanIDs.FRONT_LEFT_DRIVE_ID,
            CanIDs.FRONT_LEFT_TURNING_ID,
            CanIDs.FRONT_LEFT_ENCODER_ID,
            FRONT_LEFT_MODULE_ENCODER_OFFSET);

        _frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            CanIDs.FRONT_RIGHT_DRIVE_ID,
            CanIDs.FRONT_RIGHT_TURNING_ID,
            CanIDs.FRONT_RIGHT_ENCODER_ID,
            FRONT_RIGHT_MODULE_ENCODER_OFFSET);

        _backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            CanIDs.BACK_LEFT_DRIVE_ID,
            CanIDs.BACK_LEFT_TURNING_ID,
            CanIDs.BACK_LEFT_ENCODER_ID,
            BACK_LEFT_MODULE_ENCODER_OFFSET);

        _backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            CanIDs.BACK_RIGHT_DRIVE_ID,
            CanIDs.BACK_RIGHT_TURNING_ID,
            CanIDs.BACK_RIGHT_ENCODER_ID,
            BACK_RIGHT_MODULE_ENCODER_OFFSET);

             // Odemetry Code
             this._odometry = new SwerveDrivePoseEstimator(_kinematics, this._navx.getRotation2d(),
                                                         new SwerveModulePosition[] {
                                                            _frontLeftModule.getModulePosition(), 
                                                            _frontRightModule.getModulePosition(),
                                                            _backLeftModule.getModulePosition(), 
                                                            _backRightModule.getModulePosition()
                                                        }, 
                                                        new Pose2d(Position.StartingXPosition, 
                                                                    Position.StartingYPosition, 
                                                                    new Rotation2d())
                                                        );
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        _navx.zeroYaw();
        _odometry.resetPosition(getGyroscopeRotation(), null, null);
    }

    public Rotation2d getGyroscopeRotation() {
        if (_navx.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(_navx.getFusedHeading());
        }

        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - _navx.getYaw());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        _chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {
        // Convert the drive base vector into module vectors
        SwerveModuleState[] states = _kinematics.toSwerveModuleStates(_chassisSpeeds);
        // Normalize the wheel speeds so we aren't trying to set above the max
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        // Set each module's speed and angle
        _frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                             states[0].angle.getRadians());
        _frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, 
                              states[1].angle.getRadians());
        _backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, 
                            states[2].angle.getRadians());
        _backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, 
                             states[3].angle.getRadians());

        _odometry.update(getGyroscopeRotation(), new SwerveModulePosition[] {
                                                _frontLeftModule.getModulePosition(),
                                                _frontRightModule.getModulePosition(),
                                                _backLeftModule.getModulePosition(),
                                                _backRightModule.getModulePosition()
                                            });
        _odometry = _visionSubsystem.getVisionEstimatedRobotPose(_odometry);
    }
}

