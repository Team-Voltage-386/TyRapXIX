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
import frc.robot.subsystems.ManipulatorSubsystem;

public class ManipulatorHatchDefault extends Command {

  public ManipulatorHatchDefault() {
    requires(Robot.manipulatorSubsystem);
  }

  @Override
  protected void initialize() {
    // Opened HatchSolenoid (To Hold Starting Hatch)
    Robot.manipulatorSubsystem.setHatchSolenoidState(ManipulatorSubsystem.HATCH_SOLENOID_OPENED);
    // Folded CargoSolenoid
    Robot.manipulatorSubsystem.setModeSolenoidState(ManipulatorSubsystem.MODE_SOLENOID_HATCH);
    // Ensure that Cargo Intake Stops
    Robot.manipulatorSubsystem.setCargoIntakeSpeed(0);
  }

  @Override
  protected void execute() {
    if (OI.xboxManipControl.getRawButton(OI.INTAKE)) {
      // Opens HatchSolenoid
      Robot.manipulatorSubsystem.setHatchSolenoidState(ManipulatorSubsystem.HATCH_SOLENOID_OPENED);
    } else if (OI.xboxManipControl.getRawButton(OI.OUTAKE)) {
      // Closes HatchSolenoid
      Robot.manipulatorSubsystem.setHatchSolenoidState(ManipulatorSubsystem.HATCH_SOLENOID_CLOSED);
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
