/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.MAXSpeedArcadeDrive;
import frc.robot.commands.ResetYaw;
import frc.robot.commands.Shifter;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 * 
 * <h2>// CREATING BUTTONS</h2>
 * <p>
 * One type of button is a joystick button which is any button on a
 * <em>joystick</em>. You create one by telling it which joystick it's on and
 * which button number it is.<br>
 * <code>Joystick stick = new Joystick(port);<br />
 * Button button = new JoystickButton(stick, buttonNumber);</code>
 * </p>
 * 
 * <p>
 * There are a few additional built in buttons you can use. Additionally, by
 * subclassing Button you can create custom triggers and bind those to commands
 * the same as any other Button.
 * </p>
 * 
 * <h4>TRIGGERING COMMANDS WITH BUTTONS</h4> Once you have a button, it's
 * trivial to bind it to a button in one of three ways:
 * 
 * <p>
 * Start the command when the button is pressed and let it run the command until
 * it is finished as determined by it's isFinished method.<br>
 * <code>button.whenPressed(new ExampleCommand());</code>
 * </p>
 * 
 * <p>
 * Run the command while the button is being held down and interrupt it once the
 * button is released.<br>
 * <code>button.whileHeld(new ExampleCommand());</code>
 * </p>
 * 
 * <p>
 * Start the command when the button is released and let it run the command
 * until it is finished as determined by it's isFinished method.<br>
 * <code>button.whenReleased(new ExampleCommand());</code>
 * </p>
 */
public class OI {

  // drive user inputs
  public static final int driveLeftJoystickVertical = 1;
  public static final int driveRightJoystickHorizontal = 4;
  public static final int speedModButton = 6;

  // button inputs
  /**
   * This button is used to start the floorPickup command. the function is tied to
   * the <em>Right Bumper</em> which is ID 6
   */
  public static final int floorPickup = 6; // right bumper
  /**
   * The button runs the command to collect from player station. it's tied to the
   * <em>X Button</em> which is ID 10
   */
  public static final int cargoPlayerStationPickup = 1; // X button
  /**
   * The button changes the selector to level 1. it's tied to the <em>A
   * Button</em> which is ID 2
   */
  public static final int levelOneSelector = 2; // A button
  /**
   * The button changes the selector to level 2. it's tied to the <em>B
   * Button</em> which is ID 3
   */
  public static final int levelTwoSelector = 3; // B button
  /**
   * The button changes the selector to level 3. it's tied to the <em>Y
   * Button</em> which is ID 4
   */
  public static final int levelThreeSelector = 4; // Y button
  /**  */
  public static final int resetLevel = 8; // right trigger
  /**
   * button used to intake cargo or hatch automatically. It's tied to the <em>Left
   * Bumper</em> which is ID 5
   */
  public static final int intake = 5; // left bumper
  /**
   * button used to send cargo/hatch automatically. It's tied to the <em>Left
   * Trigger</em> which is ID 7
   */
  public static final int outake = 7; // left trigger
  /**
   * used the <em>Y Axis</em> of the <em>Left Analog Stick</em> to control the
   * shoulder's axis.
   */
  public static final int manualShoulderAxis = 1; // left joystick y
  /**
   * used the <em>Y Axis</em> of the <em>Right Analog Stick</em> to control the
   * shoulder's axis.
   */
  public static final int manualElbowAxis = 3; // right joystick y

  // manipulator mode buttons
  public static final int hatchMode = 10; // start button
  public static final int cargoMode = 9; // back button

  public static Joystick xboxDriveControl = new Joystick(RobotMap.driveControllerPort);
  public static Joystick xboxManipControl = new Joystick(RobotMap.manipControllerPort);

  Button maxSpeeedButton = new JoystickButton(xboxDriveControl, speedModButton);
  Button shifterButton = new JoystickButton(xboxDriveControl, 5);
  Button resetPigeonYawButton = new JoystickButton(xboxDriveControl, 2);

  public OI() {
    maxSpeeedButton.whileHeld(new MAXSpeedArcadeDrive());
    shifterButton.whenPressed(new Shifter());
    resetPigeonYawButton.whenPressed(new ResetYaw());
  }
}