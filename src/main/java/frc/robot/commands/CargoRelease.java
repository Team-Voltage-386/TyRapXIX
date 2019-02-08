/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

/**
 * Intended to run the motors to eject cargo without running the motors much
 * longer than necessary.
 */

public class CargoRelease extends Command {
  public static final double RELEASE_SPEED = -0.5;

  Timer timer = new Timer();
  double time;

  public CargoRelease(double timeOut) {
    this.time = timeOut;
    requires(Robot.manipulatorSubsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.manipulatorSubsystem.setCargoIntakeSpeed(RELEASE_SPEED);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return timer.get() > time;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.manipulatorSubsystem.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
