package frc.robot.drive.sds;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    SwerveModulePosition getModulePosition();

    void set(double driveVoltage, double steerAngle);
}
