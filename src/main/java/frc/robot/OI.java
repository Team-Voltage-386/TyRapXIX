package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.drive.MAXSpeedArcadeDrive;
import frc.robot.commands.manipulator.ManipulatorCargoMode;
import frc.robot.commands.manipulator.ManipulatorHatchMode;
import frc.robot.commands.drive.Shifter;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  // drive user inputs
  public static final int DRIVE_LEFT_JOYSTICK_VERTICAL = 1;
  public static final int DRIVE_RIGHT_JOYSTICK_HORIZONTAL = 4;
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

  Button cargoModeButton = new JoystickButton(xboxManipControl, CARGO_MODE);
  Button hatchModeButton = new JoystickButton(xboxManipControl, HATCH_MODE);
  Button maxSpeeedButton = new JoystickButton(xboxDriveControl, SPEED_MOD_BUTTON);
  // Temporary Button Numbers for all buttons not using RobotMap
  Button shifterButton = new JoystickButton(xboxDriveControl, 5);

  public OI() {
    maxSpeeedButton.whileHeld(new MAXSpeedArcadeDrive());
    shifterButton.whenPressed(new Shifter());
    cargoModeButton.whenPressed(new ManipulatorCargoMode());
    hatchModeButton.whenPressed(new ManipulatorHatchMode());
  }
}