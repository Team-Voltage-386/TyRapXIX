/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem.Levels;


public class HatchMode extends Command {
  Levels desiredLevel = Levels.hatchLevelOne;
  public HatchMode() {
    requires(Robot.armSubsystem);
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
    if (OI.xboxManipControl.getRawButton(RobotMap.floorPickup)){
      //floor pickup
      desiredLevel = Levels.hatchFloorPickup;
    }
    else if (OI.xboxManipControl.getRawButton(RobotMap.levelOneSelector)){ 
      //level one
      desiredLevel = Levels.hatchLevelOne;
    }
    else if (OI.xboxManipControl.getRawButton(RobotMap.levelTwoSelector)){
      //level two
      desiredLevel = Levels.hatchLevelTwo;
    }
    else if (OI.xboxManipControl.getRawButton(RobotMap.levelThreeSelector)){
      //level three 
      desiredLevel = Levels.hatchLevelThree;
    }
    else{
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
