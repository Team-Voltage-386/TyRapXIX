/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TurnDegrees extends Command {

  private double p, currentAngle, angleGoal, prevAngle = 0, angleChange;

  public TurnDegrees(double angle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSubsystem);
    angleGoal = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveSubsystem.resetPigeon();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    currentAngle = Robot.driveSubsystem.getPigeonYPR()[0];
    angleChange = currentAngle - prevAngle;
    p = (angleGoal-currentAngle)*0.015;
    Robot.driveSubsystem.driveTank(p, -1*p);
    prevAngle = currentAngle;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(currentAngle) >= Math.abs(angleGoal) && Math.abs(currentAngle) <= 1.1 * Math.abs(angleGoal) && angleChange < 0.05;
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
