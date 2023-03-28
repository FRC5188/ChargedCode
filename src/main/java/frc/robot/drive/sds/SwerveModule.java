package frc.robot.drive.sds;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();
   
    DriveController getDriveController();

    SwerveModulePosition getModulePosition();

    void set(double driveVoltage, double steerAngle);
}
