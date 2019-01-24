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
import frc.robot.subsystems.DriveSubsystem;

public class TurnLeft extends Command {

  private double degrees,p,d,prevError,errorChange;
  private double error = 99;
  private double leftSpeed,rightSpeed;
  private double pk = .02, dk = .02; //constants

  public TurnLeft() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
    this.degrees=90;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveSubsystem.resetGyro();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("Error",error);
    SmartDashboard.putNumber("Error Change",errorChange);
    SmartDashboard.putNumber("Left Speed",p+d); 
    SmartDashboard.putNumber("Right Speed",-p-d); 
    error = (degrees - Robot.driveSubsystem.getYaw());
    errorChange = error - prevError;
    p=pk*error;
    d=dk*errorChange;
    if((p < 0) && (p > -0.4)){ 
      p = -0.4;
    }
    else if((p > 0) && (p < .4)){
      p=0.4;
    }
    leftSpeed = p+d;
    rightSpeed = -p-d;
    DriveSubsystem.driveTank(leftSpeed,rightSpeed);
    prevError = error;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return ((Math.abs(Robot.driveSubsystem.getYaw()) >= degrees) && (errorChange < .2) && (Math.abs(Robot.driveSubsystem.getYaw()) < degrees*1.01));
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
