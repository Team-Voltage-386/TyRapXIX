/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
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
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  public static int driveControllerPort = 0;
  public static int manipControllerPort = 1;

  // solenoids
  public static int shifterLow = 0;
  public static int shifterHigh = 1;
  public static int hatchCaptureOpen = 2;
  public static int hatchCaptureClosed = 3;
  public static int beakRetractOpen = 4;
  public static int beakRetractClosed = 5;

  // sparks
  public static int rightClimbArm = 0;
  public static int leftClimbArm = 1;
  public static int elevatorDriveWheels = 2;

  // pigeon
  public static int pigeonPort = 3;

  // sensors
  public static int lineSensorLeft = 0;
  public static int lineSensorCenter = 1;
  public static int lineSensorRight = 2;
  public static int distanceSensorPing = 3;
  public static int distanceSensorEcho = 4;
  public static int analogUltrasonic = 0;
  public static final int ENCODER_PORT = 0;

  /*
   * drive motors There's a discrepancy between the drive motor types listed on
   * trello and what's listed on the official port mappings sheet, so I input the
   * motors and added them into the drive subsystem as they are on the port
   * mappings sheet.
   */
  public static int frontLeft = 1; // talon
  public static int frontRight = 2; // talon
  public static int rearLeftFollower = 4; // victor spx
  public static int rearRightFollower = 3; // victor spx

  public static int spikeLightRing = 0; // LED Light Ring

  // button inputs
  public static int floorPickup = 1;
  public static int cargoPlayerStationPickup = 10; // placeholder - update with final button mapping
  public static int levelOneSelector = 2;
  public static int levelTwoSelector = 3;
  public static int levelThreeSelector = 4;

  // to be defined - port numbers are placeholders
  public static int rearElevatorMotor = 5;
  public static int leftShoulderMotor = 6;
  public static int rightShoulderMotor = 7;
  public static int elbowMotor = 8;
  public static int cargoRollerMotor = 9;
  // encoders
  public static int shoulderEncoder = 0;
  public static int rearElevatorEncoder = 1;
  public static int elbowEncoder = 2;
  public static int driveLeftEncoder = 3;
  public static int driveRightEncoder = 4;
  // drive user inputs
  public static int driveLeftJoystickVertical = 1;
  public static int driveRightJoystickHorizontal = 4;
  // manip user inputs
  public static int manipLeftJoystickVertical = 1;
  public static int manipRightJoystickVertical = 5;

  // limit switches
  public static int bottomArmLimitSwitch = 4;

}
