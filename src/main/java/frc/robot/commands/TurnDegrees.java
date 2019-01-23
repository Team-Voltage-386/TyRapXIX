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

public class TurnDegrees extends Command {

  private double p, d, currentAngle, angleGoal, prevError, errorChange, leftSpeed, rightSpeed, error;
  private final double pk = 0.015, dk = 0.03;

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
    SmartDashboard.putBoolean("auto ended", false);
    prevError = 0;
    errorChange = 999999999;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    currentAngle = Robot.driveSubsystem.getPigeonYPR()[0];
    error = angleGoal-currentAngle;
    p = error*pk;
    errorChange = error - prevError;
    d = errorChange*dk;
    //leftSpeed = minimumSpeedCheck(p + d);
    //rightSpeed = minimumSpeedCheck(-p + d);
    leftSpeed = minimumSpeedCheck(p + d);
    rightSpeed = minimumSpeedCheck(-p - d);
    Robot.driveSubsystem.driveTank(leftSpeed, rightSpeed);
    prevError = error;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //return false;
    return Math.abs(currentAngle) >= Math.abs(angleGoal) && Math.abs(currentAngle) <= 1.01 * Math.abs(angleGoal) && errorChange < 0.001;
  } ///RETRUN STATEMENT SHOULD NOT USE ANGLECHANGE

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

  private double minimumSpeedCheck(double calculatedSpeed){
    if(Math.abs(calculatedSpeed) < 0.45){
      if(calculatedSpeed<0){
        calculatedSpeed=-0.45;
      }
      else{
        calculatedSpeed=0.45;
      }
    }
    return calculatedSpeed;
  }

}