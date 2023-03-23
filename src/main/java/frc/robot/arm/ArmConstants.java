package frc.robot.arm;

import frc.robot.arm.Arm.WristPosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.arm.Arm.Arm2DPosition;
import frc.robot.arm.Arm.IntakeMode;

public class ArmConstants {

        public static double SHOULDER_0_DEGREE_POT_OFFSET = 2217;
        public static double SHOULDER_90_DEGREE_POT_OFFSET = 1868;
        public static double ELBOW_90_DEGREE_POT_OFFSET = 1708;
        public static double ELBOW_0_DEGREE_POT_OFFSET = 2060;

        // All in degrees
        public static double SHOULDER_UPPER_SOFT_STOP = 150;
        public static double SHOULDER_LOWER_SOFT_STOP = 5;
        public static double ELBOW_UPPER_SOFT_STOP = 130;
        public static double ELBOW_LOWER_SOFT_STOP = -10;

        // The y,z position of the shoulder joint relative to the floor
        public static double SHOULDER_JOINT_Z_POS = 17; // inches
        public static double SHOULDER_JOINT_Y_POS = 0; // inches

        // Physical arm constants (used for feedforward and trajectory following)
        public static final double ELBOW_LENGTH = 0.781;
        public static final double ELBOW_MOI = 1.68;
        public static final double ELBOW_CGRADIUS = 0.679;
        public static final double ELBOW_MASS = 3.243;
        public static final DCMotor ELBOW_MOTOR = DCMotor.getFalcon500(1).withReduction(200);

        public static final double SHOULDER_LENGTH = 0.707;
        public static final double SHOULDER_MOI = 0.406;
        public static final double SHOULDER_CGRADIUS = 0.274;
        public static final double SHOULDER_MASS = 3.856;
        public static final DCMotor SHOULDER_MOTOR = DCMotor.getFalcon500(1).withReduction(200);

        public static class AngleSetpoints {

                /*
                 * How to update a setpoint:
                 * 1) Get the robot ready. You MUST comment out the lines letting the
                 * shoulder and elbow move (_shoulderMotor.set for example).
                 * There should only be 2 spots this happens, but I suggest searching
                 * for .set( to make sure everything is commented out. You also need to change
                 * the idle mode to coast instead of brake so you can move the arm.
                 * That gets handled in the constructor. Do for both joints.
                 * 2) Open up shuffleboard. If you go to the smart dashboard tab, you should
                 * see a bunch of numbers. What you are looking for is the ones labeled
                 * elbow angle and shoulder angle. Those should update as you move the arm.
                 * 3) Move to your new position and take note of those angles. They will
                 * get displayed in the smart dashboard.
                 * 4) Change the values in code. Make sure you pick the correct position
                 * and update the numbers for the shoulder and elbow angles you just took
                 * 5) Uncomment the stuff you commented out and change the idle mode back
                 * to brake. Go ahead and test. MAKE SURE YOU'RE READY TO DISABLE!
                 */

                public static double STORED_SHOULDER_POS = 81.7;
                public static double STORED_ELBOW_POS = 1;

                public static double MID_CONE_SHOULDER_POS = 78.1;
                public static double MID_CONE_ELBOW_POS = 91.6;

                public static double MID_CONE_SHOULDER_PLACE_POS = 78.1;
                public static double MID_CONE_ELBOW_PLACE_POS = 67.1;

                public static double MID_CUBE_SHOULDER_POS = 107.8;
                public static double MID_CUBE_ELBOW_POS = 80.7;

                public static double HIGH_CONE_SPIT_SHOULDER_POS = 40.2;
                public static double HIGH_CONE_SPIT_ELBOW_POS = 110;

                // this is set to be the height prior to dropping. ignore the name.
                // needs refactored
                public static double HIGH_CONE_DROP_SHOULDER_POS = 44.7;
                public static double HIGH_CONE_DROP_ELBOW_POS = 121.5;

                public static double HIGH_CUBE_SHOULDER_POS = 77.3;
                public static double HIGH_CUBE_ELBOW_POS = 95.0;

                public static double HUMAN_PLAYER_SHOULDER_POS = 113.5;
                public static double HUMAN_PLAYER_ELBOW_POS = 92.5;

                public static double GROUND_PICKUP_SHOULDER_POS = 46.2;
                public static double GROUND_PICKUP_ELBOW_POS = 1;

                public static double TIPPED_CONE_SHOULDER_POS = 0; // TODO
                public static double TIPPED_CONE_ELBOW_POS = 0; // TODO

                public static double INTERMEDIATE_SCORING_SHOULDER_POS = 95;
                public static double INTERMEDIATE_SCORING_ELBOW_POS = 31;

                public static double EN_GARDE_SHOULDER_POS = 147;
                public static double EN_GARDE_ELBOW_POS = 90;

                public static double INTERMEDIATE_TO_PICKUP_SHOULDER_POS = 50;
                public static double INTERMEDIATE_TO_PICKUP_ELBOW_POS = 25;

                public static double INTERMEDIATE_FROM_PICKUP_SHOULDER_POS = 87;
                public static double INTERMEDIATE_FROM_PICKUP_ELBOW_POS = 25;

                public static double INTERMEDIATE_ALL_SHOULDER_POS = 109.6;
                public static double INTERMEDIATE_ALL_ELBOW_POS = 25.7;
        }

        public static class IntermediateWaypoints {
                // Add waypoints for trajectories here
                // Each inner array is a waypoint with 2 numbers; First is the shoulder angle,
                // and second is the elbow angle
                public static double[][] STORED_TO_SCORING = { { 95, 31 } };
                public static double[][] STORED_TO_ENGARDE = { { 97, 18 }, { 113.2, 34 }, { 126.6, 46 },
                                { 136.2, 59.2 } };
                public static double[][] ENGARDE_TO_STORED = { { 131, 63.7 }, { 113.5, 39.3 }, { 97.7, 20.7 },
                                { 87.1, 22 }, { 84.1, 18 }, { 81.7, 10 } };
                public static double[][] STORED_TO_GROUND_PICKUP = { { 114, 30 }, { 103.2, 40.3 }, { 92.3, 55.5 }, { 47.5, 45 } };
                public static double[][] GROUND_PICKUP_TO_STORED = {{46.2, 7}, {69.6, 30}, {80, 3.5}};
        }
        public static class SetPoints2D {

                /**
                 * constants for the above defined in the ArmPosition Enum.
                 * This is the place to make edits to setpoints.
                 */
                private final double STORED_Y_POS = 15.75;
                private final double STORED_X_POS = -9;
                public static WristPosition STORED_WRIST_POS = WristPosition.Perpendicular;
                public final Arm2DPosition STORED_SETPOINT = new Arm2DPosition(STORED_Y_POS,
                                STORED_X_POS,
                                STORED_WRIST_POS);

                private final double GROUND_PICKUP_Y_POS = 0.0;
                private final double GROUND_PICKUP_X_POS = 0.0;
                public static WristPosition GROUND_PICKUP_WRIST_POS = WristPosition.Perpendicular;
                public final Arm2DPosition GROUND_PICKUP_SETPOINT = new Arm2DPosition(GROUND_PICKUP_Y_POS,
                                GROUND_PICKUP_X_POS,
                                GROUND_PICKUP_WRIST_POS);

                private final double TIPPED_CONE_Y_POS = 0.0;
                private final double TIPPED_CONE_X_POS = 0.0;
                public static WristPosition TIPPED_CONE_WRIST_POS = WristPosition.Perpendicular;
                public final Arm2DPosition TIPPED_CONE_SETPOINT = new Arm2DPosition(TIPPED_CONE_Y_POS,
                                TIPPED_CONE_X_POS,
                                TIPPED_CONE_WRIST_POS);

                private final double HIGH_CUBE_Y_POS = 0.0;
                private final double HIGH_CUBE_X_POS = 0.0;
                public static WristPosition HIGH_CUBE_WRIST_POS = WristPosition.Parallel;
                public final Arm2DPosition HIGH_CUBE_SETPOINT = new Arm2DPosition(HIGH_CUBE_Y_POS,
                                HIGH_CUBE_X_POS,
                                HIGH_CUBE_WRIST_POS);

                private final double HIGH_CONE_Y_POS = 0.0;
                private final double HIGH_CONE_X_POS = 0.0;
                public static WristPosition HIGH_CONE_WRIST_POS = WristPosition.Parallel;
                public final Arm2DPosition HIGH_CONE_SETPOINT = new Arm2DPosition(HIGH_CONE_Y_POS,
                                HIGH_CONE_X_POS,
                                HIGH_CONE_WRIST_POS);

                private final double LOAD_STATION_PICKUP_Y_POS = 0.0;
                private final double LOAD_STATION_PICKUP_X_POS = 0.0;
                public static WristPosition LOAD_STATION_PICKUP_WRIST_POS = WristPosition.Parallel;
                public final Arm2DPosition LOAD_STATION_PICKUP_SETPOINT = new Arm2DPosition(LOAD_STATION_PICKUP_Y_POS,
                                LOAD_STATION_PICKUP_X_POS,
                                LOAD_STATION_PICKUP_WRIST_POS);

                private final double LOW_SCORE_Y_POS = 0.0;
                private final double LOW_SCORE_X_POS = 0.0;
                public static WristPosition LOW_SCORE_WRIST_POS = WristPosition.Perpendicular;
                public final Arm2DPosition LOW_SCORE_SETPOINT = new Arm2DPosition(LOW_SCORE_Y_POS,
                                LOW_SCORE_X_POS,
                                LOW_SCORE_WRIST_POS);

                private final double MIDDLE_CONE_Y_POS = 0.0;
                private final double MIDDLE_CONE_X_POS = 0.0;
                public static WristPosition MIDDLE_CONE_WRIST_POS = WristPosition.Parallel;
                public final Arm2DPosition MIDDLE_CONE_SETPOINT = new Arm2DPosition(MIDDLE_CONE_Y_POS,
                                MIDDLE_CONE_X_POS,
                                MIDDLE_CONE_WRIST_POS);

                private final double MIDDLE_CUBE_Y_POS = 0.0;
                private final double MIDDLE_CUBE_X_POS = 0.0;
                public static WristPosition MIDDLE_CUBE_WRIST_POS = WristPosition.Parallel;
                public final Arm2DPosition MIDDLE_CUBE_SETPOINT = new Arm2DPosition(MIDDLE_CUBE_Y_POS,
                                MIDDLE_CUBE_X_POS,
                                MIDDLE_CUBE_WRIST_POS);

                public static WristPosition EN_GARDE_WRIST_POS = WristPosition.Perpendicular;

        }

        public static double ELBOW_IS_HITTING_CURRENT = 9999.0; // TODO: Find actual value for this
        public static double MAX_MOTOR_VOLTAGE = 11.5; // May want to adjust -Garrett

        public static double INTAKE_HAS_PIECE_CURRENT = 40;

        public static IntakeMode INTAKE_MODE_DEFAULT = IntakeMode.Open;

        // shoulder PID constants
        // private final double SHOULDER_MOTOR_KP = 0.015;
        // private final double SHOULDER_MOTOR_KI = 0.0005;
        // private final double SHOULDER_MOTOR_KD = 0.0005;
        public static double SHOULDER_MOTOR_KP = 0.015;
        public static double SHOULDER_MOTOR_KI = 0.0005;
        public static double SHOULDER_MOTOR_KD = 0.0005;
        public static double SHOULDER_MOTOR_TOLERANCE = 8.0;

        // shoulder motion profile constraints
        private static double SHOULDER_MAX_VELOCITY = 250; // max speed that this joint should move at
        private final static double SHOULDER_MAX_ACCELERATION = 230; // max acceleration this joint should move at
        public static TrapezoidProfile.Constraints SHOULDER_MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(
                        SHOULDER_MAX_VELOCITY,
                        SHOULDER_MAX_ACCELERATION);

        // Elbow constants
        // private final double ELBOW_MOTOR_KP = 0.02;
        // private final double ELBOW_MOTOR_KI = 0.0005;
        // private final double ELBOW_MOTOR_KD = 0.0005;
        public static double ELBOW_MOTOR_KP = 0.015;
        public static double ELBOW_MOTOR_KI = 0.0005;
        public static double ELBOW_MOTOR_KD = 0.0005;
        public static double ELBOW_MOTOR_TOLERANCE = 5.0;

        // shoulder motion profile constraints
        public static double ELBOW_MAX_VELOCITY = 250; // max speed that this joint should move at
        public static double ELBOW_MAX_ACCELERATION = 230; // max acceleration this joint should move at
        public static TrapezoidProfile.Constraints ELBOW_MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(
                        ELBOW_MAX_VELOCITY,
                        ELBOW_MAX_ACCELERATION);

        // public static double MAX_TRAJECTORY_SPEED = 50;
        public static double MAX_TRAJECTORY_SPEED = 3;
}
