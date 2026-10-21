// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// /**
//  * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
//  * constants. This class should not be used for any other purpose. All constants should be declared
//  * globally (i.e. public static). Do not put anything functional in this class.
//  *
//  * <p>It is advised to statically import this class (or one of its inner classes) wherever the
//  * constants are needed, to reduce verbosity.
//  */
// public final class Constants {
//   public static class OperatorConstants {
//     public static final int kDriverControllerPort = 0;
//   }
// }
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
  public static class BeltDexerConstants {
    public static final int MotorcanIDA = 14;
    public static final int MotorcanIDB = 15;
    public static final double gearRatioA = 1;
    public static final double gearRatioB = 3;
    public static final boolean invertA = false;
    public static final boolean invertB = false;
    public static final double MotorAP = 1;
    public static final double MotorAI = 0;
    public static final double MotorAD = 0;
    public static final double MotorBP = 1;
    public static final double MotorBI = 0;
    public static final double MotorBD = 0;
  }
}
