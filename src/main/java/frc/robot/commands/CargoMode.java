/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem.Levels;

/**
 * This command will constantly check for a specific button to be pressed and
 * will set the mode for cargo accordingly
 */
public class CargoMode extends Command {
  Levels desiredLevel = Levels.cargoLevelOne;

  public CargoMode() {
    requires(Robot.armSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (OI.xboxManipControl.getRawButton(OI.FLOOR_PICKUP)) {
      desiredLevel = Levels.cargoFloorPickup;
    } else if (OI.xboxManipControl.getRawButton(OI.CARGO_PLAYER_STATION_PICKUP)) {
      // position for collecting cargo from the human player station
      desiredLevel = Levels.cargoPlayerStation;
    } else if (OI.xboxManipControl.getRawButton(OI.LEVEL_ONE_SELECTOR)) {
      // level one
      desiredLevel = Levels.cargoLevelOne;
    } else if (OI.xboxManipControl.getRawButton(OI.LEVEL_TWO_SELECTOR)) {
      // level two
      desiredLevel = Levels.cargoLevelTwo;
    } else if (OI.xboxManipControl.getRawButton(OI.LEVEL_THREE_SELECTOR)) {
      // level three
      desiredLevel = Levels.cargoLevelThree;
    } else {
    }
    Robot.armSubsystem.setLevel(desiredLevel);
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
