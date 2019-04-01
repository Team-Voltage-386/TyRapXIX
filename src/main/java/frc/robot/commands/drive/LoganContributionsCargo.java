/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmCargoMode;
import frc.robot.commands.manipulator.ManipulatorCargoMode;

/**
 * Add your docs here.
 */
public class LoganContributionsCargo extends InstantCommand {
  /**
   * Add your docs here.
   */

  Command command1, command2;

  public LoganContributionsCargo() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSubsystem);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    command1 = new ArmCargoMode();
    command1.start();
    command2 = new ManipulatorCargoMode();
    command2.start();
  }

}
