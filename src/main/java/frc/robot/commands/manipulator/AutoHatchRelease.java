/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.ManipulatorSubsystem;

/**
 * Add your docs here.
 */
public class AutoHatchRelease extends InstantCommand {
  /**
   * Add your docs here.
   */
  public AutoHatchRelease() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.manipulatorSubsystem);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.manipulatorSubsystem.setHatchSolenoidState(ManipulatorSubsystem.HATCH_SOLENOID_CLOSED);
  }

}
