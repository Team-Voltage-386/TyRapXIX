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

public class DriveFullyForward extends Command {

  private Levels level;
  private double error, ultrasonicGoalVoltage;
  private final double DEFAULT_FORWARD_SPEED = 0.4; // TEMP NEEDS TO BE TESTED
  private final double k = 0; // TEMP NEEDS TO BE TESTED FOR GYRO COMPENSATION

  public DriveFullyForward(Levels levelInput) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSubsystem);
    level = levelInput;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveSubsystem.resetPigeon();
    error = 0;
    if (level == Levels.cargoLevelOne) {
      ultrasonicGoalVoltage = 100; // TEMP NEEDS TO BE TESTED
    } else if (level == Levels.cargoLevelTwo) {
      ultrasonicGoalVoltage = 100; // TEMP NEEDS TO BE TESTED
    } else if (level == Levels.cargoLevelThree) {
      ultrasonicGoalVoltage = 100; // TEMP NEEDS TO BE TESTED
    } else if (level == Levels.hatchLevelOne) {
      ultrasonicGoalVoltage = 100; // TEMP NEEDS TO BE TESTED
    } else if (level == Levels.hatchLevelTwo) {
      ultrasonicGoalVoltage = 100; // TEMP NEEDS TO BE TESTED
    } else if (level == Levels.hatchLevelThree) {
      ultrasonicGoalVoltage = 100; // TEMP NEEDS TO BE TESTED
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    error = Robot.driveSubsystem.getPigeonYPR()[0];
    // - and + may be backwards
    Robot.driveSubsystem.driveTank(DEFAULT_FORWARD_SPEED - (error * k), DEFAULT_FORWARD_SPEED + (error * k));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.driveSubsystem.getUltrasonicVoltage() < ultrasonicGoalVoltage;
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
