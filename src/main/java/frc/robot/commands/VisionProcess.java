/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class VisionProcess extends Command {
  double error, previousError;
  double k, i, p;
  double kp = .02, ki = 0.0, kd = 0.0;

  public VisionProcess() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.visionProcessing);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    Robot.spikeSubsystem.lightSwitch();
    error = Robot.visionProcessing.visionProcess();
    previousError = error;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    error = Robot.visionProcessing.visionProcess();

    p = error * kp;

    Robot.driveSubsystem.driveTank(-p, +p);
    SmartDashboard.putNumber("Final Error", error);
    SmartDashboard.putNumber("Error1", Robot.visionProcessing.error1);
    SmartDashboard.putNumber("Error2", Robot.visionProcessing.error2);
    SmartDashboard.putNumber("p", p);
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
