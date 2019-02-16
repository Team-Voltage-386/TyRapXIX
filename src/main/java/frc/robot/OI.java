package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.climb.DeployClimbArms;
import frc.robot.commands.drive.MAXSpeedArcadeDrive;
import frc.robot.commands.drive.Shifter;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  // drive user inputs
  public static final int DRIVE_LEFT_JOYSTICK_VERTICAL = 1;
  public static final int DRIVE_RIGHT_JOYSTICK_HORIZONTAL = 4;
  public static final int MANIP_LEFT_JOYSTICK_VERTICAL = 1;
  public static final int MANIP_RIGHT_JOYSTICK_VERTICAL = 3;
  public static final int SPEED_MOD_BUTTON = 6;
  public static final int SHIFT_BUTTON = 5;

  // button inputs
  public static final int FLOOR_PICKUP = 6; // right bumper
  public static final int CARGO_PLAYER_STATION_PICKUP = 1; // X button
  public static final int LEVEL_ONE_SELECTOR = 2; // A button
  public static final int LEVEL_TWO_SELECTOR = 3; // B button
  public static final int LEVEL_THREE_SELECTOR = 4; // Y button
  public static final int RESET_LEVEL = 8; // right trigger
  public static final int INTAKE = 5; // left bumper
  public static final int OUTAKE = 7; // left trigger
  public static final int MANUAL_SHOULDER_AXIS = 1; // left joystick y
  public static final int MANUAL_ELBOW_AXIS = 3; // right joystick y

  // manipulator mode buttons
  public static final int HATCH_MODE = 10; // start button
  public static final int CARGO_MODE = 9; // back button

  public static final Joystick xboxDriveControl = new Joystick(RobotMap.driveControllerPort);
  public static final Joystick xboxManipControl = new Joystick(RobotMap.manipControllerPort);

  private Button maxSpeeedButton = new JoystickButton(xboxDriveControl, SPEED_MOD_BUTTON);
  private Button shifterButton = new JoystickButton(xboxDriveControl, SHIFT_BUTTON);

  // THESE BUTTONS ARE ALL TEMPORARY
  Button tempClimbArmsButton = new JoystickButton(xboxManipControl, 1);
  Button tempClimbPhaseOneButton = new JoystickButton(xboxManipControl, 2);
  Button tempUltrasonicDriveElevatorWheelsButton = new JoystickButton(xboxManipControl, 3);
  Button tempLiftClimbLegsButton = new JoystickButton(xboxManipControl, 4);
  // THESE BUTTONS ARE ALL TEMPORARY

  public OI() {
    maxSpeeedButton.whileHeld(new MAXSpeedArcadeDrive());
    shifterButton.whenPressed(new Shifter());
    tempClimbArmsButton.whenPressed(new DeployClimbArms());
    // tempClimbPhaseOneButton.whenPressed(new ClimbPhaseOne());
    // tempUltrasonicDriveElevatorWheelsButton.whenPressed(new
    // UltrasonicDriveElevatorWheels(30));
    // tempLiftClimbLegsButton.whenPressed(new LiftClimbLegs());
  }
}