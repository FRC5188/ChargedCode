package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sds.Mk4iSwerveModuleHelper;
import frc.robot.sds.SdsModuleConfigurations;
import frc.robot.sds.SwerveModule;

/**
 * Singleton subsystem for Drivebase.
 */
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
    public static final double MAX_VOLTAGE = 12.0;

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
            Math.hypot(CHASSIS_WIDTH_METERS / 2.0, CHASSIS_HEIGHT_METERS / 2.0) * 0.5;

    /** The offset to get the encoder to read 0 when facing forward */
    private static final double FRONT_LEFT_MODULE_ENCODER_OFFSET = -43.501171;
    /** The offset to get the encoder to read 0 when facing forward */
    private static final double FRONT_RIGHT_MODULE_ENCODER_OFFSET = -290.390625;
    /** The offset to get the encoder to read 0 when facing forward */
    private static final double BACK_LEFT_MODULE_ENCODER_OFFSET = -142.81640625;
    /** The offset to get the encoder to read 0 when facing forward */
    private static final double BACK_RIGHT_MODULE_ENCODER_OFFSET = -352.873828125;

    private Vision _visionSubsystem;

    /**
     * This object does the math to convert a motion vector into individual module
     * vectors
     * <p>
     * See WPILib's documentation for more information
     **/
    // We divide our chassis width and height by 2 so we can represent each module
    // as a point relative to the
    // center of the robot. Positive along x means going to the left from the
    // center, and positive along
    // y means going up from the center

    private SwerveDriveKinematics _kinematics;

    /**
     * The gyro that we will use to keep track of our current rotation. The output
     * of the gyro
     * impacts the odometry of the robot, as well as field-oriented drive.
     */
    private AHRS _navx;

    private SwerveDrivePoseEstimator _odometry;
    private double _speedMultiplier = 0.4;

    // These are our modules
    private SwerveModule _frontLeftModule;
    private SwerveModule _frontRightModule;
    private SwerveModule _backLeftModule;
    private SwerveModule _backRightModule;

    /**
     * This represents the desired vector of the robot.
     * <p>
     * The first part is the forwards velocity, which is how fast we want to go forward/backward.
     * Second is the sideways velocity, which is how fast we want to strafe left/right.
     * Last is the angular velocity, which is how fast we want to rotate cw/ccw.
     * <p>
     */
    private ChassisSpeeds _chassisSpeeds;

    // Holds the instance of the drive subsystem
    private static Drive _instance = null;

    /**
     * Represents the drive chassis of the robot. Contains all of the code to
     * move in a swerve format using either a joystick or supplied values.
     */
    public Drive(Vision visionSubsystem) {
        _visionSubsystem = visionSubsystem;

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain Info");

        _frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(6, 8)
                        .withPosition(0, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.CanIDs.FRONT_LEFT_DRIVE_ID,
                Constants.CanIDs.FRONT_LEFT_TURNING_ID,
                Constants.CanIDs.FRONT_LEFT_ENCODER_ID,
                FRONT_LEFT_MODULE_ENCODER_OFFSET);

        _frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(6, 8)
                        .withPosition(6, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.CanIDs.FRONT_RIGHT_DRIVE_ID,
                Constants.CanIDs.FRONT_RIGHT_TURNING_ID,
                Constants.CanIDs.FRONT_RIGHT_ENCODER_ID,
                FRONT_RIGHT_MODULE_ENCODER_OFFSET);

        _backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(6, 8)
                        .withPosition(0, 8),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.CanIDs.BACK_LEFT_DRIVE_ID,
                Constants.CanIDs.BACK_LEFT_TURNING_ID,
                Constants.CanIDs.BACK_LEFT_ENCODER_ID,
                BACK_LEFT_MODULE_ENCODER_OFFSET);

        _backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(6, 8)
                        .withPosition(6, 8),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.CanIDs.BACK_RIGHT_DRIVE_ID,
                Constants.CanIDs.BACK_RIGHT_TURNING_ID,
                Constants.CanIDs.BACK_RIGHT_ENCODER_ID,
                BACK_RIGHT_MODULE_ENCODER_OFFSET);

        _kinematics = new SwerveDriveKinematics(
                // Front left
                new Translation2d(CHASSIS_WIDTH_METERS / 2.0, CHASSIS_HEIGHT_METERS / 2.0),
                //
                new Translation2d(CHASSIS_WIDTH_METERS / 2.0, -CHASSIS_HEIGHT_METERS / 2.0),
                // Back left
                new Translation2d(-CHASSIS_WIDTH_METERS / 2.0, CHASSIS_HEIGHT_METERS / 2.0),
                // Back right
                new Translation2d(-CHASSIS_WIDTH_METERS / 2.0, -CHASSIS_HEIGHT_METERS / 2.0));

        _navx = new AHRS();

        _chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        _odometry = new SwerveDrivePoseEstimator(_kinematics, getGyroscopeRotation(), new SwerveModulePosition[] {
                _frontLeftModule.getModulePosition(), _frontRightModule.getModulePosition(),
                _backLeftModule.getModulePosition(), _backRightModule.getModulePosition() }, new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        _navx.reset();
    }

    public static void setInstance(Vision visionSubsystem) {
        _instance = (_instance == null) ? (new Drive(visionSubsystem)) : (_instance);
    }

    public static Drive getInstance() {
        return _instance;
    }

    public AHRS getGyroInstance() {
        return _navx;
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        _navx.zeroYaw();
        _odometry.resetPosition(getGyroscopeRotation(), null, null);
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                _frontLeftModule.getModulePosition(), _frontRightModule.getModulePosition(),
                _backLeftModule.getModulePosition(), _backRightModule.getModulePosition() };
    }

    public Rotation2d getOdometryRotation2d() {
        return _odometry.getEstimatedPosition().getRotation();
    }

    public Pose2d getPose() {
        return _odometry.getEstimatedPosition();
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
    public void setSpeedMultiplier(double multiplier){
        _speedMultiplier = multiplier;
    }

    public double getSpeedMultiplier(){
        return this._speedMultiplier;
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        SwerveModulePosition modulePositionArray[] = { _frontLeftModule.getModulePosition(),
                                _frontRightModule.getModulePosition(), _backLeftModule.getModulePosition(),
                                _backRightModule.getModulePosition() };

                        _odometry.resetPosition(
                                getGyroscopeRotation(),
                                modulePositionArray,
                                traj.getInitialHolonomicPose());
                    }
                }),

                new PPSwerveControllerCommand(traj,
                        this::getPose, new PIDController(0, 0, 0),
                        new PIDController(0, 0, 0), new PIDController(0, 0, 0),
                        this::drive, this));
    };

    public void drive(ChassisSpeeds chassisSpeeds) {
        _chassisSpeeds = chassisSpeeds;
    }


    @Override
    public void periodic() {
        // Convert the drive base vector into module vectors
        SwerveModuleState[] states = _kinematics.toSwerveModuleStates(_chassisSpeeds);
        // Normalize the wheel speeds so we aren't trying to set above the max
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        // Update odometry if applicable
        _visionSubsystem.getVisionEstimatedRobotPose(_odometry);
        _odometry.updateWithTime(Timer.getFPGATimestamp(), getGyroscopeRotation(), new SwerveModulePosition[] {
                _frontLeftModule.getModulePosition(), _frontRightModule.getModulePosition(),
                _backLeftModule.getModulePosition(), _backRightModule.getModulePosition()
        });

        // Set each module's speed and angle
        _frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
        _frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
        _backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
        _backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());
    }
}
