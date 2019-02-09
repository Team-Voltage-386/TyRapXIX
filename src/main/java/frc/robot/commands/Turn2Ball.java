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

public class Turn2Ball extends Command {
  private double error, p, i, d;
  private final double pk = .012, ik = 0, dk = .015;

  public Turn2Ball() {
    requires(Robot.ballVisionSubystem);
    requires(Robot.driveSubsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    error = Robot.ballVisionSubystem.ballVision();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    i = 0;
    double previousError = error;
    error = Robot.ballVisionSubystem.ballVision();
    SmartDashboard.putNumber("error", error);
    p = error * pk;
    SmartDashboard.putNumber("p", p);
    d = (previousError - error) * dk;
    SmartDashboard.putNumber("d", d);

    Robot.driveSubsystem.driveTank(-p + d + i, +p - i - d);

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
