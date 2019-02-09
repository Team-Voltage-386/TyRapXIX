/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * <h2>The Gear Shift function</h2>
 * <p>
 * a toggle function that switches between high gear and low gear
 * </p>
 * <h3>High Gear</h3>
 * <p>
 * This is when the robot goes faster and the wheels roll freely, but it cant
 * turn without browning out
 * </p>
 * <h3>Low Gear</h3>
 * <p>
 * This is when the robot goes slower, but its more controllable. needs to be in
 * low gear to turn or move to exact positions easily
 * </p>
 */
public class Shifter extends InstantCommand {
  public Shifter() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSubsystem);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.driveSubsystem.shift();
  }

}
