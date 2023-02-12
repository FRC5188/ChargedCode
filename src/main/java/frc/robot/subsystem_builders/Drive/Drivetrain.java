package frc.robot.subsystem_builders.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.OperatorConstants.CanIDs;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.sds.Mk4iSwerveModuleHelper;
import frc.robot.sds.SdsModuleConfigurations;
import frc.robot.sds.SwerveModule;
import frc.robot.subsystems.Drive;

public abstract class Drivetrain {
    /**
    * The maximum voltage that will be delivered to the drive motors.
    * <p>
    * This can be reduced to cap the robot's maximum speed. */
    public static final double MAX_VOLTAGE = 12.0;
    /**
    * The maximum velocity of the robot in meters per second.
    * <p>
    * This is a measure of how fast the robot should be able to drive in a straight
    * line.
    * The formula for calculating the theoretical maximum velocity is:
    * <p>
    * Motor free speed RPM / 60 * Drive reduction * Wheel diameter meters * Pi */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
            SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
    /** The width of the chassis from the centers of the swerve modules */
    private static final double CHASSIS_WIDTH_METERS = Units.inchesToMeters(20.75);
    /** The height of the chassis from the centers of the swerve modules */
    private static final double CHASSIS_HEIGHT_METERS = Units.inchesToMeters(24.75);
    /** The offset to get the encoder to read 0 when facing forward */
    private static final double FRONT_LEFT_MODULE_ENCODER_OFFSET = -323.7890625;
    /** The offset to get the encoder to read 0 when facing forward */ 
    private static final double FRONT_RIGHT_MODULE_ENCODER_OFFSET = -351.73828125;
    /** The offset to get the encoder to read 0 when facing forward */
    private static final double BACK_LEFT_MODULE_ENCODER_OFFSET = -201.533203125;
    /** The offset to get the encoder to read 0 when facing forward */
    private static final double BACK_RIGHT_MODULE_ENCODER_OFFSET = -312.355390125;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(CHASSIS_WIDTH_METERS / 2.0, CHASSIS_HEIGHT_METERS / 2.0) * 0.5;
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
    private static SwerveDriveKinematics _kinematics;
    private static SwerveModule _frontLeftModule;
    private static SwerveModule _frontRightModule;
    private static SwerveModule _backLeftModule;
    private static SwerveModule _backRightModule;
/**
     * This represents the desired vector of the robot.
     * <p>
     * The first part is the forwards velocity, which is how fast we want to go forward/backward.
     * Second is the sideways velocity, which is how fast we want to strafe left/right.
     * Last is the angular velocity, which is how fast we want to rotate cw/ccw.
     * <p>
     */
    private static ChassisSpeeds _chassisSpeeds;      
    
    public static void buildDrivebase(ShuffleboardTab shuffleboardTab){
            _frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                    shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                            .withSize(6, 8)
                            .withPosition(0, 0),
                    Mk4iSwerveModuleHelper.GearRatio.L2,
                    CanIDs.FRONT_LEFT_DRIVE_ID,
                    CanIDs.FRONT_LEFT_TURNING_ID,
                    CanIDs.FRONT_LEFT_ENCODER_ID,
                    FRONT_LEFT_MODULE_ENCODER_OFFSET);
    
            _frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                    shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                            .withSize(6, 8)
                            .withPosition(6, 0),
                    Mk4iSwerveModuleHelper.GearRatio.L2,
                    CanIDs.FRONT_RIGHT_DRIVE_ID,
                    CanIDs.FRONT_RIGHT_TURNING_ID,
                    CanIDs.FRONT_RIGHT_ENCODER_ID,
                    FRONT_RIGHT_MODULE_ENCODER_OFFSET);
    
            _backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                    shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                            .withSize(6, 8)
                            .withPosition(0, 8),
                    Mk4iSwerveModuleHelper.GearRatio.L2,
                    CanIDs.BACK_LEFT_DRIVE_ID,
                    CanIDs.BACK_LEFT_TURNING_ID,
                    CanIDs.BACK_LEFT_ENCODER_ID,
                    BACK_LEFT_MODULE_ENCODER_OFFSET);
    
            _backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                    shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                            .withSize(6, 8)
                            .withPosition(6, 8),
                    Mk4iSwerveModuleHelper.GearRatio.L2,
                    CanIDs.BACK_RIGHT_DRIVE_ID,
                    CanIDs.BACK_RIGHT_TURNING_ID,
                    CanIDs.BACK_RIGHT_ENCODER_ID,
                    BACK_RIGHT_MODULE_ENCODER_OFFSET);
                    
            _kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(CHASSIS_WIDTH_METERS / 2.0, CHASSIS_HEIGHT_METERS / 2.0),
            // Front right
            new Translation2d(CHASSIS_WIDTH_METERS / 2.0, -CHASSIS_HEIGHT_METERS / 2.0),
            // Back left
            new Translation2d(-CHASSIS_WIDTH_METERS / 2.0, CHASSIS_HEIGHT_METERS / 2.0),
            // Back right
            new Translation2d(-CHASSIS_WIDTH_METERS / 2.0, -CHASSIS_HEIGHT_METERS / 2.0));

            _chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    }
    public static void buildDrivebase(){
            _frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                    Mk4iSwerveModuleHelper.GearRatio.L2,
                    CanIDs.FRONT_LEFT_DRIVE_ID,
                    CanIDs.FRONT_LEFT_TURNING_ID,
                    CanIDs.FRONT_LEFT_ENCODER_ID,
                    FRONT_LEFT_MODULE_ENCODER_OFFSET);
    
            _frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                    Mk4iSwerveModuleHelper.GearRatio.L2,
                    CanIDs.FRONT_RIGHT_DRIVE_ID,
                    CanIDs.FRONT_RIGHT_TURNING_ID,
                    CanIDs.FRONT_RIGHT_ENCODER_ID,
                    FRONT_RIGHT_MODULE_ENCODER_OFFSET);
    
            _backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                    Mk4iSwerveModuleHelper.GearRatio.L2,
                    CanIDs.BACK_LEFT_DRIVE_ID,
                    CanIDs.BACK_LEFT_TURNING_ID,
                    CanIDs.BACK_LEFT_ENCODER_ID,
                    BACK_LEFT_MODULE_ENCODER_OFFSET);
    
            _backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                    Mk4iSwerveModuleHelper.GearRatio.L2,
                    CanIDs.BACK_RIGHT_DRIVE_ID,
                    CanIDs.BACK_RIGHT_TURNING_ID,
                    CanIDs.BACK_RIGHT_ENCODER_ID,
                    BACK_RIGHT_MODULE_ENCODER_OFFSET);
                    
            _kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(CHASSIS_WIDTH_METERS / 2.0, CHASSIS_HEIGHT_METERS / 2.0),
            // Front right
            new Translation2d(CHASSIS_WIDTH_METERS / 2.0, -CHASSIS_HEIGHT_METERS / 2.0),
            // Back left
            new Translation2d(-CHASSIS_WIDTH_METERS / 2.0, CHASSIS_HEIGHT_METERS / 2.0),
            // Back right
            new Translation2d(-CHASSIS_WIDTH_METERS / 2.0, -CHASSIS_HEIGHT_METERS / 2.0));

            _chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    }
    public static void updateModules(){
            SwerveModuleState[] states = _kinematics.toSwerveModuleStates(_chassisSpeeds);
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
    }
    public static SwerveDriveKinematics getKinematics(){return _kinematics;}
    public static SwerveModule getFrontLeftModule(){return _frontLeftModule;}
    public static SwerveModule getFrontRightModule(){return _frontRightModule;}
    public static SwerveModule getBackLeftModule(){return _backLeftModule;}
    public static SwerveModule getBackRightModule(){return _backRightModule;}
    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                _frontLeftModule.getModulePosition(), _frontRightModule.getModulePosition(),
                _backLeftModule.getModulePosition(), _backRightModule.getModulePosition() };
    }
    public static void drive(ChassisSpeeds chassisSpeeds) {
        _chassisSpeeds = chassisSpeeds;
    }
    public static void setJoystickDrive(Drive driveSubsystem, XboxController driveController){
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
            driveSubsystem,
            () -> (-modifyAxis(driveController.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND),
            () -> (-modifyAxis(driveController.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND),
            () -> (-modifyAxis(driveController.getRightX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)));
    }
    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
      }
      private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);
    
        // Square the axis
        value = Math.copySign(value * value, value);
    
        return value;
      }
}
