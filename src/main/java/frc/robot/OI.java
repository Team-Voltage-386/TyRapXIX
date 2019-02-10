package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.BackUp;
import frc.robot.commands.DeployObject;
import frc.robot.commands.DriveFullyForward;
import frc.robot.commands.MAXSpeedArcadeDrive;
import frc.robot.commands.Shifter;
import frc.robot.subsystems.ArmSubsystem.Levels;
import frc.robot.commands.RunAutoLevelOne;
import frc.robot.commands.RunAutoLevelTwo;
import frc.robot.commands.RunAutoLevelThree;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  // drive user inputs
  public static final int DRIVE_LEFT_JOYSTICK_VERTICAL = 1;
  public static final int DRIVE_RIGHT_JOYSTICK_HORIZONTAL = 4;
  public static final int SPEED_MOD_BUTTON = 6;

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

  Button maxSpeeedButton = new JoystickButton(xboxDriveControl, SPEED_MOD_BUTTON);
  Button shifterButton = new JoystickButton(xboxDriveControl, 5);
  Button autoLevelOne = new JoystickButton(xboxDriveControl, 1); // TEMP
  Button autoLevelTwo = new JoystickButton(xboxDriveControl, 2); // TEMP
  Button autoLevelThree = new JoystickButton(xboxDriveControl, 3); // TEMP

  Button tempDriveFullyForward = new JoystickButton(xboxManipControl, 1);
  Button tempDeployObject = new JoystickButton(xboxManipControl, 2);
  Button tempBackUp = new JoystickButton(xboxManipControl, 3);

  public OI() {
    maxSpeeedButton.whileHeld(new MAXSpeedArcadeDrive());
    shifterButton.whenPressed(new Shifter());
    autoLevelOne.whenPressed(new RunAutoLevelOne());
    autoLevelTwo.whenPressed(new RunAutoLevelTwo());
    autoLevelThree.whenPressed(new RunAutoLevelThree());
    // ultrasonic distance needs to be measured for each level by Voltage
    tempDriveFullyForward.whenPressed(new DriveFullyForward(Levels.cargoLevelOne)); // needs to test each scoring level
    // manually shift solenoid state? or only test one of them
    tempDeployObject.whenPressed(new DeployObject());
    // Test clearance distance with ultrasonic
    tempBackUp.whenPressed(new BackUp());
  }
}