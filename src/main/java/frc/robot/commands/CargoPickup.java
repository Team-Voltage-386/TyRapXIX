/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robot;

/**
 * Intended to run the motors to pickup cargo without running the motors much
 * longer than necessary. You may want to change it later to run the motors to
 * keep a grip on cargo as needed.
 */

public class CargoPickup extends Command {
  public static final double INTAKE_SPEED = 0.5;

  Timer timer = new Timer();
  double time;

  public CargoPickup(double timeIn) {
    this.time = timeIn;
    requires(Robot.manipulatorSubsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time

  /**
   * .set statements open the beak to the proper position before cargo is picked
   * up
   */
  @Override
  protected void initialize() {
    Robot.manipulatorSubsystem.setHatchSolenoidState(Value.kForward);
    Robot.manipulatorSubsystem.setCargoSolenoidState(Value.kReverse);

    timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.manipulatorSubsystem.setCargoIntakeSpeed(INTAKE_SPEED);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return timer.get() > time;
  }

  // Called once after isFinished returns true
  /**
   * May want to change the "stop" speed to just be slower than the inital speed
   * later so the motors continue to run to keep a grip or whatever is necessary.
   */
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
