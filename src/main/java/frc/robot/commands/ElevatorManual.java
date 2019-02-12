/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.OI;

public class ElevatorManual extends Command {
  public ElevatorManual() {
    requires(Robot.endgameClimbSubsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    /**
     * Joystick up (negative speed) moves elevator down, Joystick down (positive
     * speed) moves elevator up
     */
    Robot.endgameClimbSubsystem.setElevatorSpeed(OI.xboxManipControl.getRawAxis(OI.manipLeftJoystickVertical));
    /** Direction needs to be tested */
    Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(OI.xboxManipControl.getRawAxis(OI.manipRightJoystickVertical));
    /** Positive speed should deploy arms, negative retracts */
    Robot.endgameClimbSubsystem.setClimbArmSpeeds(OI.xboxDriveControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
