/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {

  public static WPI_TalonSRX leftShoulder = new WPI_TalonSRX(RobotMap.leftShoulderMotor);
  public static WPI_TalonSRX rightShoulder = new WPI_TalonSRX(RobotMap.rightShoulderMotor);

  public static WPI_TalonSRX elbowMotor = new WPI_TalonSRX(RobotMap.elbowMotor);

  public ArmSubsystem() {
    leftShoulder.follow(rightShoulder);
    leftShoulder.setInverted(InvertType.OpposeMaster);
  }

  public enum Levels {
    cargoFloorPickup, cargoPlayerStation, cargoLevelOne, cargoLevelTwo, cargoLevelThree, hatchFloorPickup,
    hatchLevelOne, hatchLevelTwo, hatchLevelThree;
  }

  public void shoulderManual() {
    leftShoulder.set(OI.xboxManipControl.getRawAxis(RobotMap.manipLeftJoystickVertical));
    // just needs to be a way to drive the motors from joystick inputs (getRawAxis)
    // add in limit switches after that
  }

  public void elbowManual() {

  }

  public void setLevel(Levels in) {
    switch (in) {
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
