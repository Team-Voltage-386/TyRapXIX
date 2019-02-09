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
import frc.robot.commands.Shifter;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
  // drive user inputs
  public static final int driveLeftJoystickVertical = 1;
  public static final int driveRightJoystickHorizontal = 4;
  public static final int speedModButton = 6;

  // button inputs
  public static final int floorPickup = 6; // right bumper
  public static final int cargoPlayerStationPickup = 1; // X button
  public static final int levelOneSelector = 2; // A button
  public static final int levelTwoSelector = 3; // B button
  public static final int levelThreeSelector = 4; // Y button
  public static final int resetLevel = 8; // right trigger
  public static final int intake = 5; // left bumper
  public static final int outake = 7; // left trigger
  public static final int manualShoulderAxis = 1; // left joystick y
  public static final int manualElbowAxis = 3; // right joystick y

  // manipulator mode buttons
  public static final int hatchMode = 10; // start button
  public static final int cargoMode = 9; // back button

  public static Joystick xboxDriveControl = new Joystick(RobotMap.driveControllerPort);
  public static Joystick xboxManipControl = new Joystick(RobotMap.manipControllerPort);

  Button maxSpeeedButton = new JoystickButton(xboxDriveControl, speedModButton);
  Button shifterButton = new JoystickButton(xboxDriveControl, 5);

  public OI() {
    maxSpeeedButton.whileHeld(new MAXSpeedArcadeDrive());
    shifterButton.whenPressed(new Shifter());
  }
}