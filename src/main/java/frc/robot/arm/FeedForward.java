package frc.robot.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.arm.DoubleJointedArmFeedforward.JointConfig;

public abstract class FeedForward {

    private static final JointConfig shoulderJoint = new JointConfig(ArmConstants.SHOULDER_MASS,
            ArmConstants.SHOULDER_LENGTH, ArmConstants.SHOULDER_MOI, ArmConstants.SHOULDER_CGRADIUS,
            ArmConstants.SHOULDER_MOTOR);
    private static final JointConfig elbowJoint = new JointConfig(ArmConstants.ELBOW_MASS, ArmConstants.ELBOW_LENGTH,
            ArmConstants.ELBOW_MOI, ArmConstants.ELBOW_CGRADIUS, ArmConstants.ELBOW_MOTOR);

    private static final DoubleJointedArmFeedforward doubleJointedArmFeedfoward = new DoubleJointedArmFeedforward(
            shoulderJoint, elbowJoint);

    public static double shoulder(double upperSetpoint, double lowerSetpoint) {
        double ff = (calculate(upperSetpoint, lowerSetpoint).get(0, 0)) / 12.0;
        SmartDashboard.putNumber("Shoulder FF", ff);
        return ff;
    }

    public static double elbow(double upperSetpoint, double lowerSetpoint) {
        double ff = (calculate(upperSetpoint, lowerSetpoint).get(1, 0)) / 12.0;
        SmartDashboard.putNumber("Elbow FF", ff);
        return ff;
    }

    private static Vector<N2> calculate(double upperSetpoint, double lowerSetpoint) {
        /*
         * read the whitepaper
         * (https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060)
         * for the
         * feedforward code, and the shoulder needs to be 0 at positive x axis and
         * increase CCW (this is how we have it now)
         * The elbow needs to be relative to the shoulder, which is not how we measure
         * the elbow normally. So we need to
         * have a different position method that doesn't make the elbow angle change
         * when the shoulder angle changes. That
         * should just be returning the directly calculated angle instead of the one
         * that uses the shoulder angle
         */

        double inputUpper = Math.toRadians(upperSetpoint);
        double inputLower = Math.toRadians(lowerSetpoint);
        Vector<N2> angles = VecBuilder.fill(inputLower, inputUpper);
        Vector<N2> vectorFF = doubleJointedArmFeedfoward.calculate(angles);
        return vectorFF;
    }
}
