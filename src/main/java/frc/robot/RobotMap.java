/*----------------------------------------------------------------------------*/
/* Copyright (c) 2007-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
   // Talons
   public static int leftPrimaryDriveMotor = 0; // motor
   public static int rightPrimaryDriveMotor = 0; // motor
   public static int leftFollowerDriveMotor = 0; // motor
   public static int rightFollowerDriveMotor = 0; // motor

   public static int leftDriveEncoderChannelA = 0; // encoder
   public static int leftDriveEncoderChannelB = 0; // encoder
   public static int rightDriveEncoderChannelA = 0; // encoder
   public static int rightDriveEncoderChannelB = 0; // encoder

  // PCM (Pneumatic Control Module)
  public static int compressor = 0;

  public static int gearShiftSolenoidForwardChannel = 0; // solenoid
  public static int gearShiftSolenoidReverseChannel = 0; // solenoid

}
