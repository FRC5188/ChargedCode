package frc.robot;

public final class Constants {

    public static class Arm {

        public static double SHOULDER_0_DEGREE_POT_OFFSET = 2217;
        public static double SHOULDER_90_DEGREE_POT_OFFSET = 1868;
        public static double ELBOW_90_DEGREE_POT_OFFSET = 1708;
        public static double ELBOW_0_DEGREE_POT_OFFSET = 2060;

        // All in degrees
        public static double SHOULDER_UPPER_SOFT_STOP = 115;
        public static double SHOULDER_LOWER_SOFT_STOP = 5;
        public static double ELBOW_UPPER_SOFT_STOP = 130;
        public static double ELBOW_LOWER_SOFT_STOP = -10;

        // The y,z position of the shoulder joint relative to the floor
        public static double SHOULDER_JOINT_Z_POS = 17; // inches
        public static double SHOULDER_JOINT_Y_POS = -15; // inches

        // arm segments lengths
        public static double SHOULDER_ARM_LENGTH = 28; // inches
        public static double ELBOW_ARM_LENGTH = 28.5; // inches

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
        public static double STORED_ELBOW_POS = 0;

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

        public static double INTERMEDIATE_SCORING_SHOULDER_POS = 95;
        public static double INTERMEDIATE_SCORING_ELBOW_POS = 31;

        public static double INTERMEDIATE_TO_PICKUP_SHOULDER_POS = 50;
        public static double INTERMEDIATE_TO_PICKUP_ELBOW_POS = 25;

        public static double INTERMEDIATE_FROM_PICKUP_SHOULDER_POS = 87;
        public static double INTERMEDIATE_FROM_PICKUP_ELBOW_POS = 25;

        public static double INTERMEDIATE_ALL_SHOULDER_POS = 109.6;
        public static double INTERMEDIATE_ALL_ELBOW_POS = 25.7;
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public final class ButtonMappings {
        /*
         * The class that holds the button mappings for an X-box controller
         * Reference the all-caps name in code to use the buttons
         * You should not need to add anything to this class, so don't touch please!
         */
        public static final int A_BUTTON = 1;
        public static final int B_BUTTON = 2;
        public static final int X_BUTTON = 3;
        public static final int Y_BUTTON = 4;
        public static final int LEFT_BUMPER = 5;
        public static final int RIGHT_BUMPER = 6;
        public static final int BACK_BUTTON = 7;
        public static final int START_BUTTON = 8;
        public static final int LEFT_JOY_BUTTON = 9;
        public static final int RIGHT_JOY_BUTTON = 10;
    }

    public static class AIO {
        public final static int ELBOW_PORT_POT = 0;
        public final static int SHOULDER_PORT_POT = 1;
    }

    public static class CanIDs {
        public static final int FRONT_LEFT_DRIVE_ID = 1;
        public static final int FRONT_LEFT_TURNING_ID = 2;
        public static final int FRONT_LEFT_ENCODER_ID = 3;

        public static final int FRONT_RIGHT_DRIVE_ID = 4;
        public static final int FRONT_RIGHT_TURNING_ID = 5;
        public static final int FRONT_RIGHT_ENCODER_ID = 6;

        public static final int BACK_LEFT_DRIVE_ID = 7;
        public static final int BACK_LEFT_TURNING_ID = 8;
        public static final int BACK_LEFT_ENCODER_ID = 9;

        public static final int BACK_RIGHT_DRIVE_ID = 10;
        public static final int BACK_RIGHT_TURNING_ID = 11;
        public static final int BACK_RIGHT_ENCODER_ID = 12;

        public static final int ARM_SHOULDER_MOTOR_ID = 13;
        public static final int ARM_ELBOW_MOTOR_ID = 14;

        public static final int ARM_INTAKE_MOTOR_ID = 15; // Change this value

        public static final int CANDLE_ID = 20;

    }

    public static class PHPorts {
        public static int WRIST_SOLENOID_PORT = 0;
    }

    /**
     * Only put constants in this class if they are subsystem-specific but it would
     * be hard to keep them in the subsystem
     */
    public static class AssortedConstants {
        /**
         * This value is the amount of time in seconds that we want the robot to take to
         * get from 0 to max speed, this will help make the robot easier to drive
         */
        public static final double DRIVE_RAMP_SECONDS = 0;
    }

    /**
     * {@summary} For information about the 2023 FRC ChargedUp Field
     */
    public static class Field {
        /**
         * {@summary} For information about the Apriltags on the field.
         */
        public static class AprilTag {
            public static final String APRIL_TAG_FIELD_MAP_PATH = "2023_april_tag_map/2023_april_tag_map.json";
        }
    }
}
