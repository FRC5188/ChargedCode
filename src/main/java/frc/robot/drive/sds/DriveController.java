package frc.robot.drive.sds;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    double getEncoderValue();
    double getDriveEncoderPosition();
}