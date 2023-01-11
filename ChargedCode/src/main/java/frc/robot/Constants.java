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
    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    private static final double FALCON_FREE_SPEED = 6380.0;
    private static final double DRIVE_REDUCTION = 1;
    private static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double MAX_VELOCITY = (FALCON_FREE_SPEED / 60.0) * DRIVE_REDUCTION * WHEEL_DIAMETER * Math.PI;

    public static final int kDriverControllerPort = 0;
  }
}
