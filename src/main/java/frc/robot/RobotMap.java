package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  /**
   * USB port number that our driver's controller is on on the drive station. It's
   * static and never changes from USB port 0
   */
  public static int driveControllerPort = 0;
  /**
   * USB Port number for the manipulator controller on the driver station. the
   * port will always be USB port 1
   */
  public static int manipControllerPort = 1;

  // solenoids
  public static int shifterLow = 6;
  public static int shifterHigh = 7;
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
  public static int distancePing = 3;
  public static int distanceEcho = 4;
  public static int analogUltrasonic = 0;
  public static final int ENCODER_PORT = 0;

  /** Front left motor Talon Device ID: 1 */
  public static int frontLeft = 1; // talon
  /** Front right motor Talon Device ID: 2 */
  public static int frontRight = 2; // talon
  /** Rear left follower motor Talon Device ID: 4 */
  public static int rearLeftFollower = 4; // talon
  /** Rear right follower motor Victor SPX Device ID: 3 */
  public static int rearRightFollower = 3; // victor spx

  public static int spikeLightRing = 0; // LED Light Ring

  // to be defined - port numbers are placeholders

  public static int rearElevatorMotor = 5;
  public static int leftShoulderMotor = 6;
  public static int rightShoulderMotor = 7;
  public static int elbowMotor = 8;
  public static int cargoRollerMotor = 9; // change to cargoCapture
  // encoders
  public static int shoulderEncoder = 0;
  public static int rearElevatorEncoder = 1;
  public static int elbowEncoder = 2;

  public static int driveLeftEncoder = 3;
  public static int driveRightEncoder = 4;

  // limit switches
  public static int bottomArmLimitSwitch = 4;

}
