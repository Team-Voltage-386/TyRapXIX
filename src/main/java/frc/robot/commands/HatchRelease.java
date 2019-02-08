/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class HatchRelease extends Command {

  public HatchRelease() {
    requires(Robot.manipulatorSubsystem);
  }

  // Called just before this Command runs the first time
  /**
   * .switch statement simply ensures the beak does not end up in the 4th spread
   * and open position that was to be avoided.
   */
  @Override
  protected void initialize() {
    Robot.manipulatorSubsystem.switchCargoSolenoidStateClosed();
  }

  // Called repeatedly when this Command is scheduled to run
  /** Checks solenoid position and switches to closed only if it's open. */
  @Override
  protected void execute() {
    Robot.manipulatorSubsystem.switchHatchSolenoidStateClosed();
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
