package frc.robot.sds;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();
}
