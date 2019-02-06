/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.BeakSubsystem.HatchScoringStates;
import frc.robot.RobotMap;
import frc.robot.OI;
import edu.wpi.first.wpilibj.command.Command;

// put the set state method IN the if statement so it only sets it when called

/**
 * Add your docs here.
 */
public class HatchIntake extends Command {
  // I believe this needs to be a way to trigger the opening and closing of the
  // beak

  public HatchIntake() {
    requires(Robot.beakSubsystem);
  }

  // Make this return true when this Command no longer needs to run execute()

  @Override
  protected void execute() {
    if (OI.xboxManipControl.getRawButton(RobotMap.beakTriggerOpen)) {
      Robot.beakSubsystem.setState(HatchScoringStates.beakOpen);
    } else if (OI.xboxManipControl.getRawButton(RobotMap.beakTriggerClosed)) {
      Robot.beakSubsystem.setState(HatchScoringStates.beakRelease);
    } else {
    }

  }

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