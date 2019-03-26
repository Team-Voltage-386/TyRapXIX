/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmHatchMode;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.manipulator.ManipulatorHatchMode;

/**
 * Add your docs here.
 */
public class AutoResumeManualControl extends InstantCommand {
  /**
   * Add your docs here.
   */

  Command command;

  public AutoResumeManualControl() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSubsystem);
    requires(Robot.armSubsystem);
    requires(Robot.manipulatorSubsystem);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    command = new ArcadeDrive();
    command.start();
    command = new ArmHatchMode();
    command.start();
    command = new ManipulatorHatchMode();
    command.start();
  }

}
