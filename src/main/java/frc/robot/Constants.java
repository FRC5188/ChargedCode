// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import org.apache.commons.lang3.ObjectUtils.Null;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 *
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 *
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * 
 * It is advised to statically import this class (or one of its inner classes)
 *
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;

  public static class Position {
    public static final double StartingXPosition = 0;
    public static final double StartingYPosition = 0;

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
    }

    /**
     * {@summary} For vision subsystem. 
     */
    public static class Vision {
        /**
         * {@summary} For information about our camera model. 
         */
        public static class Camera {
            /*
            Assume that you're looking at the robot from above it. In our code we treat the robot like a single point in an XY-Plane. Where the front of the robot is the
            positive X, and where the left side of the robot is the negative Y. 
            */
            // How far foward/backward the camera is from robot center. 
            private static final double CAMERA_X_FROM_ROBOT_CENTER = 0;
            // How far left/right the camera is from robot center. 
            private static final double CAMERA_Y_FROM_ROBOT_CENTER = 0;
            // How far up/down the camera is from center if we look at robot from side in 3D space. 
            private static final double CAMERA_Z_FROM_ROBOT_CENTER = 0;
            private static final double CAMERA_ROLL = 0;
            private static final double CAMERA_PITCH = 0;
            private static final double CAMERA_YAW = 0;

            /**
            * {@summary} Represents the difference between where our camera is an where the robot is. 
            */
            public static final Transform3d DIFFERENCE_BETWEEN_ROBOT_CAMERA = new Transform3d(
                new Translation3d(CAMERA_X_FROM_ROBOT_CENTER, CAMERA_Y_FROM_ROBOT_CENTER, CAMERA_Z_FROM_ROBOT_CENTER),
                new Rotation3d(CAMERA_ROLL, CAMERA_PITCH, CAMERA_YAW)
            );
        }
        /**
         * {@summary} For information about our PhotonVision Pipeline
         */
        public static class PhotonVision {
            /**
            * {@summary} Represents the name of the camera set in the PhotonVision local utiliy. 
            */
            public static final String PHOTON_CAMERA_NAME = "photoncamera";

            
        }
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
}
