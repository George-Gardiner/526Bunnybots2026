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
    public static final double MotorASpeed = 0.3;
    public static final double MotorBSpeed = 0.9;
  }
  public static class SlapdownConstants {
    public static final double rollerVelocity = 3000;
    public static final double rollerP = 0.0001;
    public static final double rollerI = 0.000001;
    public static final double rollerD = 0;
    public static final int pivotMotorLeftID = 16;
    public static final int pivotMotorRightID = 17;
    public static final int rollerMotorID = 18;
    public static final boolean invertPivot = false;
    public static final boolean invertRoller = true;
    public static final int pivotCurrentLimit = 40;
    public static final int rollerCurrentLimit = 40;
    public static final double pivotP = 0.003;
    public static final double pivotI = 0;
    public static final double pivotD = 0;
    public static final double pivotKcos = .8; //in voltage
    public static final double pivotStowAngle = 10;
    public static final double pivotL1Angle = 70;
    public static final double pivotTransferAngle = 110;
    public static final double pivotGroundAngle = 140;
    public static final double horizontalAngle = 134.5;
    public static final double rollerDutyCycleSpeed = 0.1;
    public static final int pivotGearRatio = 60;
    public static final int rollerGearRatio = 3;
    public static final double rollerRelativeEncoderVelocityConversionFactor = 1.0/rollerGearRatio;;
    public static final double pivotRelativeEncoderVelocityConversionFactor = 1.0/pivotGearRatio;
    public static final double pivotRelativeEncoderPositionConversionFactor = 360.0/pivotGearRatio;
    public static final double pivotAbsoluteEncoderPositionConversionFactor = 360.0;
  } 
}
