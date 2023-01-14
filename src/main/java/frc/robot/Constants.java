// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Position {
    public static final double StartingXPosition = 0;
    public static final double StartingYPosition = 0;

  }

  public static class CanIDs {
    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_LEFT_TURNING_ID = 2;
    public static final int FRONT_LEFT_TURNING_ENCODER_ID = 3;
    public static final int FRONT_LEFT_DRIVE_ENCODER_ID = 999999;



    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int FRONT_RIGHT_TURNING_ID = 5;
    public static final int FRONT_RIGHT_TURNING_ENCODER_ID = 6;
    public static final int FRONT_RIGHT_DRIVE_ENCODER_ID = 999999;


    public static final int BACK_LEFT_DRIVE_ID = 7;
    public static final int BACK_LEFT_TURNING_ID = 8;
    public static final int BACK_LEFT_TURNING_ENCODER_ID = 9;
    public static final int BACK_LEFT_DRIVE_ENCODER_ID = 999999;


    public static final int BACK_RIGHT_DRIVE_ID = 10;
    public static final int BACK_RIGHT_TURNING_ID = 11;
    public static final int BACK_RIGHT_TURNING_ENCODER_ID = 12;
    public static final int BACK_RIGHT_DRIVE_ENCODER_ID = 9999;

  }
}
