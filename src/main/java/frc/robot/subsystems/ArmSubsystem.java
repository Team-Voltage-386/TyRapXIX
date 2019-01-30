/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {

  public enum Levels {
    cargoFloorPickup,
    cargoPlayerStation,
    cargoLevelOne,
    cargoLevelTwo,
    cargoLevelThree,
    hatchFloorPickup,
    hatchLevelOne,
    hatchLevelTwo,
    hatchLevelThree;
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public void setLevel(Levels in) {
    switch(in) {
      case cargoFloorPickup:
        break;
      case cargoPlayerStation:
        break;
      case cargoLevelOne:
        break;
      case cargoLevelTwo:
        break;
      case cargoLevelThree:
        break;
      case hatchFloorPickup:
        break; 
      case hatchLevelOne:
        break;
      case hatchLevelTwo:
        break;
      case hatchLevelThree:
        break;
      default: 
        break;
    }
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

