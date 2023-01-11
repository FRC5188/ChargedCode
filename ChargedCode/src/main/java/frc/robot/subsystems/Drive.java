package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    // Width and height of the robot chassis with the corners at the center of each swerve module
    // We need these numbers to complete swerve math
    private final double CHASSIS_WIDTH = Units.inchesToMeters(26);
    private final double CHASSIS_HEIGHT = Units.inchesToMeters(30);

    // These represent the centers of the swerve modules relative to the center of the robot
    private Translation2d _frontLeftLocation;
    private Translation2d _frontRightLocation;
    private Translation2d _backLeftLocation;
    private Translation2d _backRightLocation;

    // This holds all of the swerve info and does a bunch of the swerve math
    private SwerveDriveKinematics _kinematics;

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

        // Creating the kinematics object using the module locations
        _kinematics = new SwerveDriveKinematics(_frontLeftLocation, _frontRightLocation, _backLeftLocation, _backRightLocation);
    }

    @Override
    public void periodic() {

    }

}
