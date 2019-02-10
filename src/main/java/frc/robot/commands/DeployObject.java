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

  Timer timer = new Timer();
  double startTime;

  public DeployObject() {
    startTime = Timer.getFPGATimestamp();
    requires(Robot.manipulatorSubsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.manipulatorSubsystem.getCargoSolenoidState() == Value.kForward) {
      Robot.manipulatorSubsystem.setHatchSolenoidState(Value.kReverse);
    } else if (Robot.manipulatorSubsystem.getCargoSolenoidState() == Value.kReverse) {
      Robot.manipulatorSubsystem.setCargoIntakeSpeed(-.5);
    } else {

    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > 5 || false; // TEMP - make constant
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
