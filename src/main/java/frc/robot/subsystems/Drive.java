package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    // Width and height of the robot chassis with the corners at the center of each swerve module
    // We need these numbers to complete swerve math
    private final double CHASSIS_WIDTH = Units.inchesToMeters(20.75);
    private final double CHASSIS_HEIGHT = Units.inchesToMeters(24.75);

    // These represent the centers of the swerve modules relative to the center of the robot
    private Translation2d _frontLeftLocation;
    private Translation2d _frontRightLocation;
    private Translation2d _backLeftLocation;
    private Translation2d _backRightLocation;

    private SwerveModuleState _frontLeftModule;
    private SwerveModuleState _frontRightModule;
    private SwerveModuleState _backLeftModule;
    private SwerveModuleState _backRightModule;

    

    // This holds all of the swerve info and does a bunch of the swerve math
    private SwerveDriveKinematics _kinematics;
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

        _frontLeftModule = new SwerveModuleState();
        _frontRightModule = new SwerveModuleState();
        _backLeftModule = new SwerveModuleState();
        _backRightModule = new SwerveModuleState();

        // Creating the kinematics object using the module locations
        _kinematics = new SwerveDriveKinematics(_frontLeftLocation, _frontRightLocation, _backLeftLocation, _backRightLocation);
        // Create a reusable container for holding our speed info
        _chassisSpeeds = new ChassisSpeeds();
    }

    @Override
    public void periodic() {

    }

    public void drive(double vx, double vy, double omega) {
        // Update speed info based on inputs
        _chassisSpeeds.vxMetersPerSecond = vx;
        _chassisSpeeds.vyMetersPerSecond = vy;
        _chassisSpeeds.omegaRadiansPerSecond = omega;

        // Convert speeds into swerve velocity
        SwerveModuleState[] states = _kinematics.toSwerveModuleStates(_chassisSpeeds);

        // Set the new swerve velocities
        _frontLeftModule = states[0];
        _frontRightModule = states[1];
        _backLeftModule = states[2];
        _backRightModule = states[3];
    }
}
