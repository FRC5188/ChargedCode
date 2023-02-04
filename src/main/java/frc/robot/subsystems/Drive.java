package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanIDs;
import frc.robot.subsystems.SwerveModule;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonVersion;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Drive extends SubsystemBase {
    // Width and height of the robot chassis with the corners at the center of each swerve module
    // We need these numbers to complete swerve math
    private final double CHASSIS_WIDTH = Units.inchesToMeters(20.75);
    private final double CHASSIS_HEIGHT = Units.inchesToMeters(24.75);

    PhotonCamera camera;

    private AHRS m_gyro;


    // The encoders will likely not be exactly at 0 when the wheel is pointed forward, so this compensates for that
    private final double FRONT_LEFT_ENCODER_OFFSET = 0;
    private final double FRONT_RIGHT_ENCODER_OFFSET = 0;
    private final double BACK_LEFT_ENCODER_OFFSET = 0;
    private final double BACK_RIGHT_ENCODER_OFFSET = 0;

    // These represent the centers of the swerve modules relative to the center of the robot
    private Translation2d _frontLeftLocation;
    private Translation2d _frontRightLocation;
    private Translation2d _backLeftLocation;
    private Translation2d _backRightLocation;

    // These hold the calculated velocity and angle for each swerve module
    private SwerveModuleState _frontLeftModuleState;
    private SwerveModuleState _frontRightModuleState;
    private SwerveModuleState _backLeftModuleState;
    private SwerveModuleState _backRightModuleState;


    // These are the actual swerve modules, we mainly set their speeds and angles using info in SwerveModuleState
    private SwerveModule _frontLeftModule;
    private SwerveModule _frontRightModule;
    private SwerveModule _backLeftModule;
    private SwerveModule _backRightModule;

    // This does the swerve math
    private SwerveDriveKinematics _kinematics;

    SwerveDrivePoseEstimator _odometry;
    Pose2d _pose;

    // This helps us convert from generic velocities into swerve velocities
    private ChassisSpeeds _chassisSpeeds;

    public Drive() {
        // Need the distances from the center of the robot to each swerve in x and y, so half width and half height is used
        double halfWidth = CHASSIS_WIDTH / 2;
        double halfHeight = CHASSIS_HEIGHT / 2;

        // Locations for the swerve drive modules relative to the robot center.
        // Think of each module as being in a quadrant on a 2D graph with center as (0, 0) when looking at the points
        _frontLeftLocation = new Translation2d(halfWidth, halfHeight);
        _frontRightLocation = new Translation2d(halfWidth, -halfHeight);
        _backLeftLocation = new Translation2d(-halfWidth, halfHeight);
        _backRightLocation = new Translation2d(-halfWidth, -halfHeight);

        PhotonCamera camera = new PhotonCamera("photonvision");

        _frontLeftModuleState = new SwerveModuleState();
        _frontRightModuleState = new SwerveModuleState();
        _backLeftModuleState = new SwerveModuleState();
        _backRightModuleState = new SwerveModuleState();

        CANCoder backRightAbsoluteEncoder = new CANCoder(Constants.CanIDs.BACK_RIGHT_DRIVE_ID);
        CANCoder frontLeftAbsoluteEncoder = new CANCoder(Constants.CanIDs.FRONT_LEFT_DRIVE_ID);
        CANCoder frontRightAbsoluteEncoder = new CANCoder(Constants.CanIDs.FRONT_RIGHT_DRIVE_ID);
        CANCoder backLeftAbsoluteEncoder = new CANCoder(Constants.CanIDs.BACK_LEFT_DRIVE_ID);


        // Each module needs a drive motor, a turning motor, an encoder, and the encoder's offset from 0
        _frontLeftModule = new SwerveModule(Constants.CanIDs.FRONT_LEFT_DRIVE_ENCODER_ID, 
                                            Constants.CanIDs.FRONT_LEFT_DRIVE_ID, 
                                            Constants.CanIDs.FRONT_LEFT_TURNING_ID, 
                                            Constants.CanIDs.FRONT_LEFT_TURNING_ENCODER_ID, 
                                            FRONT_LEFT_ENCODER_OFFSET);
        _frontRightModule = new SwerveModule(Constants.CanIDs.FRONT_RIGHT_DRIVE_ENCODER_ID, 
                                            Constants.CanIDs.FRONT_RIGHT_DRIVE_ID, 
                                            Constants.CanIDs.FRONT_RIGHT_TURNING_ID, 
                                            Constants.CanIDs.FRONT_RIGHT_TURNING_ENCODER_ID, 
                                            FRONT_RIGHT_ENCODER_OFFSET);
         _backLeftModule = new SwerveModule(Constants.CanIDs.BACK_LEFT_DRIVE_ENCODER_ID, 
                                            Constants.CanIDs.BACK_LEFT_DRIVE_ID, 
                                            Constants.CanIDs.BACK_LEFT_TURNING_ID, 
                                            Constants.CanIDs.BACK_LEFT_TURNING_ENCODER_ID, 
                                            BACK_LEFT_ENCODER_OFFSET);
        _backRightModule = new SwerveModule(Constants.CanIDs.BACK_RIGHT_DRIVE_ENCODER_ID,
                                            Constants.CanIDs.BACK_RIGHT_DRIVE_ID, 
                                            Constants.CanIDs.BACK_RIGHT_TURNING_ID, 
                                            Constants.CanIDs.BACK_RIGHT_DRIVE_ENCODER_ID, 
                                            BACK_RIGHT_ENCODER_OFFSET);


                                    

        // Create the kinematics object using the module locations
        _kinematics = new SwerveDriveKinematics(_frontLeftLocation, _frontRightLocation, _backLeftLocation, _backRightLocation);
        SwerveDrivePoseEstimator _odometry = new SwerveDrivePoseEstimator(_kinematics, m_gyro.getRotation2d(), new SwerveModulePosition[] {_frontLeftModule.getPosition(), _frontRightModule.getPosition(), _backLeftModule.getPosition(), _backRightModule.getPosition()}, new Pose2d(Constants.Position.StartingXPosition, Constants.Position.StartingYPosition, new Rotation2d()));
    }

    public void updateOdometry() {
        _odometry.update(
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
              _frontLeftModule.getPosition(),
              _frontRightModule.getPosition(),
              _backLeftModule.getPosition(),
              _backRightModule.getPosition()
            });
      }

    @Override
    public void periodic() {
       
      }

    public void drive(double vx, double vy, double omega) {
        // Update speed info based on inputs
        _chassisSpeeds.vxMetersPerSecond = vx;
        _chassisSpeeds.vyMetersPerSecond = vy;
        _chassisSpeeds.omegaRadiansPerSecond = omega;

        // Convert speeds into swerve velocity (basically do the fancy swerve math)
        SwerveModuleState[] states = _kinematics.toSwerveModuleStates(_chassisSpeeds);

        // Update the swerve info and optimize it so that the modules don't flip around everywhere
        _frontLeftModuleState = states[0];
        _frontRightModuleState = states[1];
        _backLeftModuleState = states[2];
        _backRightModuleState = states[3];



        // Set each module's velocity and angle based on swerve info
        _frontLeftModule.set(_frontLeftModuleState);
        _frontRightModule.set(_frontRightModuleState);
        _backLeftModule.set(_backLeftModuleState);
        _backRightModule.set(_backRightModuleState);
    }
}