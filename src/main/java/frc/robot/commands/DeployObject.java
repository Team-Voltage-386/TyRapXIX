/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class DeployObject extends Command {

  private double startTime;
  private final double CARGO_OUTTAKE_TIME = 3; // TEMP NEEDS TO BE TESTED

  public DeployObject() {
    requires(Robot.manipulatorSubsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.manipulatorSubsystem.getCargoSolenoidState() == Value.kForward) { // May be backwards
      Robot.manipulatorSubsystem.setHatchSolenoidState(Value.kReverse); // May be backwards
    } else if (Robot.manipulatorSubsystem.getCargoSolenoidState() == Value.kReverse) { // May be backwards
      Robot.manipulatorSubsystem.setCargoIntakeSpeed(-.5);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > CARGO_OUTTAKE_TIME
        || Robot.manipulatorSubsystem.getCargoSolenoidState() == Value.kForward;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.manipulatorSubsystem.setCargoIntakeSpeed(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
