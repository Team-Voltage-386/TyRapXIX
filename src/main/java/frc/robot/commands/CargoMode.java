/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmSubsystem.CargoLevels;


public class CargoMode extends Command {
  CargoLevels desiredLevel = CargoLevels.cargoLevelOne; 
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
    if (Robot.m_oi.xboxManipControl.getRawButton(RobotMap.floorPickup)){
      desiredLevel = CargoLevels.cargoFloorPickup;
    }
    else if (Robot.m_oi.xboxManipControl.getRawButton(RobotMap.levelOneSelector)){ 
      //level one 
      desiredLevel = CargoLevels.cargoLevelOne; 
    }
    else if (Robot.m_oi.xboxManipControl.getRawButton(RobotMap.levelTwoSelector)){
      //level two
      desiredLevel = CargoLevels.cargoLevelTwo;
    }
    else if (Robot.m_oi.xboxManipControl.getRawButton(RobotMap.levelThreeSelector)){
      //level three
      desiredLevel = CargoLevels.cargoLevelThree;
    }
    else {
    }
    Robot.armSubsystem.setCargoLevel(desiredLevel);
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
