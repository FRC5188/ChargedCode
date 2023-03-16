package frc.robot.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.arm.DoubleJointedArmFeedforward.JointConfig;

public abstract class FeedForward {
    private static final double ELBOW_LENGTH = 0.781;
    private static final double ELBOW_MOI = 1.68;
    private static final double ELBOW_CGRADIUS = 0.679;
    private static final double ELBOW_MASS = 3.243;
    private static final DCMotor ELBOW_MOTOR = DCMotor.getFalcon500(1).withReduction(200);

    private static final double SHOULDER_LENGTH = 0.707;
    private static final double SHOULDER_MOI = 0.406;
    private static final double SHOULDER_CGRADIUS = 0.274;
    private static final double SHOULDER_MASS = 3.856;
    private static final DCMotor SHOULDER_MOTOR = DCMotor.getFalcon500(1).withReduction(200);

    private static final JointConfig shoulderJoint = new JointConfig(SHOULDER_MASS, SHOULDER_LENGTH, SHOULDER_MOI, SHOULDER_CGRADIUS, SHOULDER_MOTOR);
    private static final JointConfig elbowJoint = new JointConfig(ELBOW_MASS, ELBOW_LENGTH, ELBOW_MOI, ELBOW_CGRADIUS, ELBOW_MOTOR);

    private static final DoubleJointedArmFeedforward doubleJointedArmFeedfoward = new DoubleJointedArmFeedforward(shoulderJoint, elbowJoint);

    public static double shoulder(double upperSetpoint, double lowerSetpoint){
        return (calculate(upperSetpoint, lowerSetpoint).get(0, 0)) / 12.0;
    }

    public static double elbow(double upperSetpoint, double lowerSetpoint){
        return (calculate(upperSetpoint, lowerSetpoint).get(1, 0)) / 12.0;
    }

    private static Vector<N2> calculate(double upperSetpoint, double lowerSetpoint) {
        double inputUpper = Math.toRadians(-upperSetpoint + 180);
        double inputLower = Math.toRadians(lowerSetpoint - 90);
        Vector<N2> angles = VecBuilder.fill(inputLower, inputUpper);
        Vector<N2> vectorFF = doubleJointedArmFeedfoward.calculate(angles);
        return vectorFF;
      }
}
