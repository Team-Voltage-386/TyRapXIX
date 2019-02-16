/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimbPhaseOne extends Command {

  private double error;
  private final double K = 0; // TEMP THIS CONSTANT NEEDS TO BE GOTTEN BY TUNING
  private final double DEFAULT_ARM_SPEED = 0.5; // TEMP THIS SPEED NEEDS TO BE TESTED
  private final double DEFAULT_ELEVATOR_SPEED = 0.5; // TEMP THIS SPEED NEEDS TO BE TESTED

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
    error = Robot.driveSubsystem.getPigeonYPR()[1]; // Both drive and climb use Pigeon
    Robot.endgameClimbSubsystem.setClimbArmSpeeds(DEFAULT_ARM_SPEED + (K * error));
    Robot.endgameClimbSubsystem.setElevatorSpeed(DEFAULT_ELEVATOR_SPEED - (K * error));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.endgameClimbSubsystem.getElevatorLimitSwitch(); // TEMP MAY BE BACKWARDS DEPENDING ON LIMITSWITCH
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
