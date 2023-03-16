// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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
        public static final int CANdleID = 20; // Katie's note to self: correct this later.
    }

    public static class PHPorts {
        public static int WRIST_SOLENOID_PORT = 0;
    }

    /**
     * Only put constants in this class if they are subsystem-specific but it would be hard to keep them in the subsystem
     */
    public static class AssortedConstants {
        /** This value is the amount of time in seconds that we want the robot to take to get from 0 to max speed, this will help make the robot easier to drive */
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
