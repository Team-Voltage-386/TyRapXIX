/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem.Levels;

public class LiftArms extends Command {

  private Levels level;
  private final double MAX_POWER_FOR_ISFINISHED = 1; // TEMP NEEDS TO BE TESTED
  private final double MAX_SPEED_FOR_ISFINISHED = 1; // TEMP NEEDS TO BE TESTED

  public LiftArms(Levels levelInput) {
    requires(Robot.armSubsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    level = levelInput;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.armSubsystem.setLevel(level);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.armSubsystem.getShoulderSpeed() < MAX_SPEED_FOR_ISFINISHED
        && Robot.armSubsystem.getShoulderPower() < MAX_POWER_FOR_ISFINISHED;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.armSubsystem.setElbowMotorSpeed(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
