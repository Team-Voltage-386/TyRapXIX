/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class MoveForwardUltrasonic extends Command {

  private double p, error, rightSpeed, leftSpeed, ultraGoalInches;

  public MoveForwardUltrasonic(double goal) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSubsystem);
    ultraGoalInches = goal;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putBoolean("auto ended", false);
    Robot.driveSubsystem.resetPigeon();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    error = -1 * Robot.driveSubsystem.getPigeonYPR()[0];
    p = error * -0.015;
    leftSpeed = -0.5 - p;
    rightSpeed = -0.5 + p;
    Robot.driveSubsystem.driveTank(leftSpeed, rightSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.driveSubsystem.getUltraDistance()<ultraGoalInches;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putBoolean("auto ended", true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
