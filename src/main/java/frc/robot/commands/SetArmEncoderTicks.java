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

public class SetArmEncoderTicks extends Command {

  private double encoderGoal, speed, error, p, i, d, prevError, errorChange;
  private final double pk = 0.05, ik = 0.001, dk = 0.07;

  public SetArmEncoderTicks(double goal) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.armSubsystem);
    encoderGoal = goal;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    error = 0;
    prevError = 0;
    p=0;
    i=0;
    d=0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    error = Robot.armSubsystem.getArmEncoderValue() - encoderGoal;
    errorChange = error - prevError;
    d = errorChange * dk /*SmartDashboard.getNumber("dk ", 0)*/;
    p = error * pk /*SmartDashboard.getNumber("pk ", 0)*/;
    i += error * ik /*SmartDashboard.getNumber("ik ", 0)*/;
    speed = p + i + d;
    Robot.armSubsystem.setArmMotorSpeed(speed);
    SmartDashboard.putNumber("ArmMotorSpeed", speed);
    prevError = error;
    if(!(Robot.armSubsystem.getBottomLimitSwitch())){
      Robot.armSubsystem.resetEncoder();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
    //return !(Robot.armSubsystem.getTopLimitSwitch());
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