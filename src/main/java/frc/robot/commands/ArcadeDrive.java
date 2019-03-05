/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ArcadeDrive extends Command {
  public ArcadeDrive() {
    requires(Robot.driveSubsystem);
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
    double xSpeed = OI.xboxDriveControl.getRawAxis(RobotMap.driveLeftJoystickVertical);
    double zRotation = OI.xboxDriveControl.getRawAxis(RobotMap.driveRightJoystickHorizontal);
    Robot.driveSubsystem.driveArcade(xSpeed, -zRotation);
    SmartDashboard.putNumber("xSpeed", OI.xboxDriveControl.getRawAxis(RobotMap.driveLeftJoystickVertical));
    SmartDashboard.putNumber("zRotation", OI.xboxDriveControl.getRawAxis(RobotMap.driveRightJoystickHorizontal));
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
