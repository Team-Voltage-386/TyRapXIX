/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;

public class DriveForward extends Command {

  private double leftSpeed;
  private double rightSpeed;

  private double ticksRequired;
  private double maxError;
  private double error;
  private double k = .01; //constant
  


  public DriveForward() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSubsystem);
    this.leftSpeed = -.5;
    this.rightSpeed = -.5;
    ticksRequired = 5000;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveSubsystem.resetGyro();
    Robot.driveSubsystem.resetEncoders();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    leftSpeed = -.5;
    rightSpeed = -.5;
    error = Robot.driveSubsystem.getYaw();
    double p = k*error*-1;
    Robot.driveSubsystem.driveTank(leftSpeed+p,rightSpeed-p);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(Robot.driveSubsystem.getRightEncoder()) > Math.abs(ticksRequired));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveSubsystem.resetEncoders();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
