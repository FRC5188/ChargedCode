package frc.robot.sds;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();
}
