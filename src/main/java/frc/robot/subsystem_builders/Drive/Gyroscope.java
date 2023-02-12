package frc.robot.subsystem_builders.Drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class Gyroscope{
    /**
    * The gyro that we will use to keep track of our current rotation. The output
    * of the gyro
    * impacts the odometry of the robot, as well as field-oriented drive.
    */
    private static AHRS _navx;
    public static void buildGyroscope(){
            _navx = new AHRS();
            _navx.reset();
    }
    public static Rotation2d getGyroscopeRotation() {
            if (_navx.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(_navx.getFusedHeading());
            }
            // We have to invert the angle of the NavX so that rotating the robot
            // counter-clockwise makes the angle increase.
            return Rotation2d.fromDegrees(360.0 - _navx.getYaw());
    }
    public static void reset(){_navx.reset();}
    public static void zeroYaw(){_navx.zeroYaw();}
}