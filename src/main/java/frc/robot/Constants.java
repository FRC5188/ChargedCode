// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class CanIDs {
    public static final int ARM_ELBOW_MOTOR_CANID = 9999;
    public static final int ARM_SHOULDER_MOTOR_CANID = 9999;

  }
  /**
   * @summary ArmPositions represents the states in which the arm will be in. 
   */
  public enum ArmPositions 
  { 
    Stored, 
    /* GAME PIECE PICKUP */
    LoadStationPickUp,
    GroundPickUpCone,

    /* GAME PIECE SCORING */
    // High Goal
    HighCone,
    HighCube,
    // Middle Goal
    MiddleCone,
    MiddleCube,
    // Low Goal
    LowCone,
    LowCube
  }
}
