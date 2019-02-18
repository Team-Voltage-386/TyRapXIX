/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.OI;
import frc.robot.Robot;

public class ManipulatorHatchMode extends Command {

  public ManipulatorHatchMode() {
    requires(Robot.manipulatorSubsystem);
  }

  @Override
  protected void initialize() {
    // Closed HatchSolenoid
    Robot.manipulatorSubsystem.setHatchSolenoidState(DoubleSolenoid.Value.kForward);
    // Not-Folded CargoSolenoid
    Robot.manipulatorSubsystem.setCargoSolenoidState(DoubleSolenoid.Value.kReverse);
    // Ensure that Cargo Intake Stops
    Robot.manipulatorSubsystem.setCargoIntakeSpeed(0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (OI.xboxManipControl.getRawButton(OI.INTAKE)) {
      // Opens HatchSolenoid
      Robot.manipulatorSubsystem.setHatchSolenoidState(DoubleSolenoid.Value.kForward);
    } else if (OI.xboxManipControl.getRawButton(OI.OUTAKE)) {
      // Closes HatchSolenoid
      Robot.manipulatorSubsystem.setHatchSolenoidState(DoubleSolenoid.Value.kReverse);
    }
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}