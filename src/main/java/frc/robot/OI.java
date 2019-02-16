package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.drive.MAXSpeedArcadeDrive;
import frc.robot.commands.drive.Shifter;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  // Both Controllers
  public static final int LEFT_JOYSTICK_VERTICAL = 1;
  public static final int RIGHT_JOYSTICK_HORIZONTAL = 4;

  // Drive User Inputs
  public static final int SPEED_MOD_BUTTON = 6;
  public static final int SHIFT_BUTTON = 5;

  // Manipulator User Inputs
  public static final int FLOOR_PICKUP = 6; // right bumper
  public static final int CARGO_PLAYER_STATION_PICKUP = 1; // X button
  public static final int LEVEL_ONE_SELECTOR = 2; // A button
  public static final int LEVEL_TWO_SELECTOR = 3; // B button
  public static final int LEVEL_THREE_SELECTOR = 4; // Y button
  public static final int RESET_LEVEL = 8; // right trigger
  public static final int INTAKE = 5; // left bumper
  public static final int OUTAKE = 7; // left trigger
  public static final int HATCH_MODE = 10; // start button
  public static final int CARGO_MODE = 9; // back button

  // Controller Instantiation
  public static final Joystick xboxDriveControl = new Joystick(RobotMap.DRIVE_CONTROLLER_PORT);
  public static final Joystick xboxManipControl = new Joystick(RobotMap.MANIP_CONTROLLER_PORT);

  // Buttons
  private Button maxSpeeedButton = new JoystickButton(xboxDriveControl, SPEED_MOD_BUTTON);
  private Button shifterButton = new JoystickButton(xboxDriveControl, SHIFT_BUTTON);

  public OI() {
    // Button Listeners
    maxSpeeedButton.whileHeld(new MAXSpeedArcadeDrive());
    shifterButton.whenPressed(new Shifter());
  }
}