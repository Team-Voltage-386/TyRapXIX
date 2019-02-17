/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.opencv.core.Rect;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Turn2Ball extends Command {
  Rect rightRect;
  double rightCenter;
  double error;
  double y;
  double effect;
  double previousCenter;
  private double p;
  private double previousError = 0;
  private double i = 0;
  private double d = 0;
  private final double kp = .0018, ki = 0, kd = .008;

  public Turn2Ball() {
    requires(Robot.ballVisionSubystem);
    requires(Robot.driveSubsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    rightRect = Robot.ballVisionSubystem.ballVision();
    rightCenter = rightRect.x + rightRect.width / 2;
    previousError = rightCenter - 160;
    previousCenter = rightCenter;
    // error = Robot.ballVisionSubystem.ballVision();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    rightRect = Robot.ballVisionSubystem.ballVision();
    rightCenter = rightRect.x + rightRect.width / 2;
    if ((error == 0) & (rightCenter == 0)) {
      error = previousCenter - 160;
    } else {
      previousCenter = rightCenter;
      error = rightCenter - 160;
      previousError = error;

    }

    p = error * kp;
    d = (previousError - error) * kd;
    if (d > 6) {
      d = 5;
    }
    SmartDashboard.putNumber("d", d);
    y = rightRect.y;
    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("Centerx", rightCenter);
    SmartDashboard.putNumber("Y Value", y);
    SmartDashboard.putNumber("p", p);
    if (rightCenter == 0) {
      Robot.driveSubsystem.driveTank(p + d, -p - d);
    } else {
      Robot.driveSubsystem.driveTank(.5 + p + d + i, .5 + -p - i - d);

    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if ((y >= 170) & (Math.abs(error) < 10)) {
      return true;
    } else {
      return false;
    }
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
