package frc.robot.arm;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.arm.Arm.Arm2DPosition;
import frc.robot.arm.Arm.ArmJointAngles;

/**
 * Thank you to 1741 Red Alert for this code!
 */
public class ArmTrajectory {
    private ArrayList<ArmJointAngles> m_waypoints; // List of waypoints for the trajectory
    private double m_maxTranslationalSpeed; // Inches per second
    private double m_totalTime; // Seconds
    private double m_totalLength; // Inches
    private ArrayList<Double> m_waypointDists; // Cumulative distance from the beginning to each waypoint
    private ArrayList<Double> m_waypointTimes; // Cumulative time from the beginning to each waypoint

    /**
     * 
     * @param waypoints ArrayList of ArmPose waypoints
     */
    public ArmTrajectory(ArrayList<ArmJointAngles> waypoints) {
        m_waypoints = new ArrayList<ArmJointAngles>(waypoints);

        // Calculate characteristics of the trajectory
        m_maxTranslationalSpeed = ArmConstants.MAX_TRAJECTORY_SPEED;
        m_totalLength = calcTotalDistance(m_waypoints);
        System.out.println("LENGTH: " + m_totalLength);
        m_totalTime = m_totalLength / m_maxTranslationalSpeed;

        // Calculate the cumulative distance and time for each of the waypoints
        m_waypointDists = new ArrayList<Double>();
        m_waypointTimes = new ArrayList<Double>();

        m_waypointDists.add(0.0);
        m_waypointTimes.add(0.0);

        for (int i = 1; i < m_waypoints.size(); i++) {
            double deltaDist = dist(m_waypoints.get(i - 1), m_waypoints.get(i));
            double deltaTime = deltaDist / m_maxTranslationalSpeed;
            m_waypointDists.add(m_waypointDists.get(i - 1) + deltaDist);
            m_waypointTimes.add(m_waypointTimes.get(i - 1) + deltaTime);
        }

        SmartDashboard.putString("Waypoints", m_waypoints.toString());
        SmartDashboard.putString("WaypointDistances", m_waypointDists.toString());
        SmartDashboard.putString("WaypointTimes", m_waypointTimes.toString());
        SmartDashboard.putNumber("TotalTime", m_totalTime);
        SmartDashboard.putNumber("TotalLength", m_totalLength);
    }

    /**
     * By intepolating between the given waypoints
     * 
     * @param time Time in seconds since the beginning of the trajectory
     * @return ArmPose (x inches, y inches, wrist rotation degrees)
     */
    public ArmJointAngles sample(double time) {
        if (time < 0.0) {
            return m_waypoints.get(0);
        }
        if (time > m_totalTime) {
            return m_waypoints.get(m_waypoints.size() - 1);
        }

        // At any point in time, the arm will be between two waypoints
        // Find which waypoints it is between
        int waypoint0 = 0;
        int waypoint1 = 0;
        for (int i = 1; i < m_waypoints.size(); i++) {
            if ((m_waypointTimes.get(i - 1) <= time) &&
                    (m_waypointTimes.get(i) >= time)) {
                waypoint0 = i - 1;
                waypoint1 = i;
                break;
            }
        }

        // Calculate the total x and y translation between the two waypoints, and how
        // long it should take
        double changeShoulder = m_waypoints.get(waypoint1).getShoulderJointAngle()
                - m_waypoints.get(waypoint0).getShoulderJointAngle();
        double changeElbow = m_waypoints.get(waypoint1).getElbowJointAngle()
                - m_waypoints.get(waypoint0).getElbowJointAngle();
        double changeTime = m_waypointTimes.get(waypoint1) - m_waypointTimes.get(waypoint0);

        // Calculate the amount of time since waypoint0
        double deltaTime = time - m_waypointTimes.get(waypoint0);

        double deltaShoulder = changeShoulder * (deltaTime / changeTime);
        double deltaElbow = changeElbow * (deltaTime / changeTime);

        ArmJointAngles targetPose = new ArmJointAngles(
                m_waypoints.get(waypoint0).getShoulderJointAngle() + deltaShoulder,
                m_waypoints.get(waypoint0).getElbowJointAngle() + deltaElbow);

        return targetPose;
    }

    private double calcTotalDistance(ArrayList<ArmJointAngles> waypoints) {
        double totalLength = 0.0;
        for (int i = 0; i < waypoints.size() - 1; i++) {
            double dist = dist(waypoints.get(i), waypoints.get(i + 1));
            System.out.println("Distance " + i + ": " + dist);
            totalLength += dist;
        }
        return totalLength;
    }

    private double dist(ArmJointAngles angles0, ArmJointAngles angles1) {
        Pose2d pose0 = arm2DPositionFromAngles(angles0.getShoulderJointAngle(), angles0.getElbowJointAngle());
        Pose2d pose1 = arm2DPositionFromAngles(angles1.getShoulderJointAngle(), angles1.getElbowJointAngle());
        System.out.println("1: " + pose0 + " 2: " + pose1);

        return Math.sqrt(Math.pow((pose1.getX() - pose0.getX()), 2.0) +
                Math.pow((pose1.getY() - pose0.getY()), 2.0));
    }

    public double getTotalLength() {
        return m_totalLength;
    }

    public double getTotalTime() {
        return m_totalTime;
    }

    /**
     * This method will do the math to calculate the wrist position in 2D space from
     * a given shoulder and
     * elbow joint angle. This method still needs a current wristposition passed
     * into it so a proper Arm2DPosition
     * can be returned.
     * 
     * @param currentShoulder The current shoulder angle. Use
     *                        {@link getShoulderJointAngle} and not the pot value.
     * @param currentElbow    the current elbow anlge. Use
     *                        {@link getElbowJointAngle} and not the pot value.
     * @return A Arm2DPosition that represents the current point in 2D space of the
     *         wrist of the arm, including the wrist state.
     */
    public static Pose2d arm2DPositionFromAngles(double currentShoulder, double currentElbow) {
        // TODO check this math
        //elbow 0 is perp with ground. shoulder is || with ground
        double y = ArmConstants.SHOULDER_JOINT_Y_POS;
        y += ArmConstants.SHOULDER_ARM_LENGTH * Math.cos(Math.toRadians(currentShoulder));
        y += ArmConstants.ELBOW_ARM_LENGTH * Math.sin(Math.toRadians(currentElbow));

        double z = ArmConstants.SHOULDER_JOINT_Z_POS;
        z += ArmConstants.SHOULDER_ARM_LENGTH * Math.sin(Math.toRadians(currentShoulder));
        z -= ArmConstants.ELBOW_ARM_LENGTH * Math.cos(Math.toRadians(currentElbow));

        System.out.println("y: " + y + " z: " + z);

        return new Pose2d(y, z, null);
    }
}
