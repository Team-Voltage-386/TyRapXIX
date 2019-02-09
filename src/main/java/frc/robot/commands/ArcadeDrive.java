/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

/**
 * setup used to run the driver controller in arcade drive. in this mode,
 * <em>Left Analog Y Axis</em> goes forward, and <em>Right Analog X Axis</em>
 * (Left/Right) turn the robot
 */
public class ArcadeDrive extends Command {
  private static final double SPEEEED_REDUCTION = 0.8;

  public ArcadeDrive() {
    requires(Robot.driveSubsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Called repeatedly when this Command is scheduled to run

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double xSpeed = OI.xboxDriveControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL);
    double zRotation = OI.xboxDriveControl.getRawAxis(OI.DRIVE_RIGHT_JOYSTICK_HORIZONTAL);
    Robot.driveSubsystem.driveArcade(xSpeed * -1 * SPEEEED_REDUCTION, zRotation);
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
