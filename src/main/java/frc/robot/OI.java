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

  public static Joystick xboxDriveControl = new Joystick(RobotMap.driveControllerPort);
  public static Joystick xboxManipControl = new Joystick(RobotMap.manipControllerPort);

  Button shifterButton = new JoystickButton(xboxDriveControl, 5);
  Button resetPigeonYawButton = new JoystickButton(xboxDriveControl, 2);

  public OI() {
    shifterButton.whenPressed(new Shifter());
    resetPigeonYawButton.whenPressed(new ResetYaw());
  }
}
