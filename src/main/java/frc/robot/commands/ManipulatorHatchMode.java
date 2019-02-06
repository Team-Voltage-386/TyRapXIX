/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.OI;
import frc.robot.Robot;

public class ManipulatorHatchMode extends Command {

  private boolean prevState;

  public ManipulatorHatchMode() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.manipulatorSubsystem);
    prevState = false;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.manipulatorSubsystem.setHatchSolenoidState(DoubleSolenoid.Value.kForward);
    Robot.manipulatorSubsystem.setCargoSolenoidState(DoubleSolenoid.Value.kForward); // This Solenoid State may be
                                                                                     // backwards
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (OI.xboxManipControl.getRawButton(1) && !prevState) { // TEMPORARY BUTTON PORT NUMBER
      Robot.manipulatorSubsystem.switchHatchSolenoidState();
      prevState = true;
    } else if (!OI.xboxManipControl.getRawButton(1)) { // TEMPORARY BUTTON PORT NUMBER
      prevState = false;
    }
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
