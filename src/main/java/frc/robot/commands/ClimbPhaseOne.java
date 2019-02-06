/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimbPhaseOne extends Command {

  private double error;
  private final double k = 0;

  public ClimbPhaseOne() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.endgameClimbSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    error = Robot.driveSubsystem.getPigeonYPR()[1];
    Robot.endgameClimbSubsystem.setClimbArmSpeeds(0.5 + (k * error)); // 0.5 is an arbitrary value for now
    Robot.endgameClimbSubsystem.setClimbLegSpeed(0.5 - (k * error)); // 0.5 is an arbitrary value for now
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.endgameClimbSubsystem.getClimbLegsLimitSwitch();
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
