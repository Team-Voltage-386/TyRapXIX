package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  // Controllers
  public static final int DRIVE_CONTROLLER_PORT = 0;
  public static final int MANIP_CONTROLLER_PORT = 1;

  // Pigeon
  public static final int PIGEON_PORT = 3;

  // Spike
  public static final int SPIKE_LIGHT_RIGHT = 0; // LED Light Ring

  // limit switches
  public static final int BOTTOM_SHOULDER_LIMIT_SWITCH = 4;

  // Sensors
  public static final int ANALOG_ULTRASONIC = 0;
  public static final int ENCODER_PORT = 0;
  public static final int POTENTIOMETER = 1;

  // Solenoids
  // Drive
  public static final int SHIFTER_FORWARD = 6;
  public static final int SHIFTER_REVERSE = 7;
  // Manipulator
  public static final int HATCH_SOLENOID_FORWARD = 2;
  public static final int HATCH_SOLENOID_REVERSE = 3;
  public static final int CARGO_SOLENOID_FORWARD = 4;
  public static final int CARGO_SOLENOID_REVERSE = 5;

  // Motor Controllers
  // Drive
  public static final int FRONT_LEFT = 1; // talon
  public static final int FRONT_RIGHT = 2; // talon
  public static final int REAR_LEFT_FOLLOWER = 4; // talon
  public static final int REAR_RIGHT_FOLLOWER = 3; // victor spx
  // Climb
  public static final int REAR_ELEVATOR_MOTOR = 5; // talon
  public static final int RIGHT_CLIMB_ARM = 0; // spark
  public static final int LEFT_CLIMB_ARM = 1; // spark
  public static final int ELEVATOR_DRIVE_WHEELS = 2; // spark
  // Arm
  public static final int ELBOW_MOTOR = 8; // talon
  public static final int SHOULDER_MOTOR = 6; // talon
  // Manipulator
  public static final int CARGO_INTAKE_MOTOR = 9; // talon

}
