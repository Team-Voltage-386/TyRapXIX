/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {

  private double prevError, error, errorChange, speed, power, p, i, d;
  private final double pk = 0.05, ik = 0.001, dk = 0.07;
  private final int CARGO_FLOOR_TICKS = 100;
  private final int CARGO_FLOOR_ANGLE = 100;
  private final int CARGO_PLAYER_STATION_TICKS = 100;
  private final int CARGO_PLAYER_STATION_ANGLE = 100;
  private final int CARGO_LEVEL_ONE_TICKS = 100;
  private final int CARGO_LEVEL_ONE_ANGLE = 100;
  private final int CARGO_LEVEL_TWO_TICKS = 100;
  private final int CARGO_LEVEL_TWO_ANGLE = 100;
  private final int CARGO_LEVEL_THREE_TICKS = 100;
  private final int CARGO_LEVEL_THREE_ANGLE = 100;
  private final int HATCH_FLOOR_TICKS = 100;
  private final int HATCH_FLOOR_ANGLE = 100;
  private final int HATCH_LEVEL_ONE_TICKS = 100;
  private final int HATCH_LEVEL_ONE_ANGLE = 100;
  private final int HATCH_LEVEL_TWO_TICKS = 100;
  private final int HATCH_LEVEL_TWO_ANGLE = 100;
  private final int HATCH_LEVEL_THREE_TICKS = 100;
  private final int HATCH_LEVEL_THREE_ANGLE = 100;

  WPI_TalonSRX armMotorMaster = new WPI_TalonSRX(RobotMap.leftShoulderMotor);
  WPI_TalonSRX armMotorFollower = new WPI_TalonSRX(RobotMap.rightShoulderMotor);

  DigitalInput bottomLimitSwitch = new DigitalInput(RobotMap.bottomArmLimitSwitch);

  public ArmSubsystem() {
    armMotorFollower.follow(armMotorMaster);
    prevError = 0;
    p = 0;
    i = 0;
    d = 0;

  }

  public enum Levels {
    cargoFloorPickup, cargoPlayerStation, cargoLevelOne, cargoLevelTwo, cargoLevelThree, hatchFloorPickup,
    hatchLevelOne, hatchLevelTwo, hatchLevelThree;
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public void setLevel(Levels in) {
    switch (in) {
    case cargoFloorPickup:
      setArmTicks(CARGO_FLOOR_TICKS);
      setElbowRotation(CARGO_FLOOR_ANGLE);
      break;
    case cargoPlayerStation:
      setArmTicks(CARGO_PLAYER_STATION_TICKS);
      setElbowRotation(CARGO_PLAYER_STATION_ANGLE);
      break;
    case cargoLevelOne:
      setArmTicks(CARGO_LEVEL_ONE_TICKS);
      setElbowRotation(CARGO_LEVEL_ONE_ANGLE);
      break;
    case cargoLevelTwo:
      setArmTicks(CARGO_LEVEL_TWO_TICKS);
      setElbowRotation(CARGO_LEVEL_TWO_ANGLE);
      break;
    case cargoLevelThree:
      setArmTicks(CARGO_LEVEL_THREE_TICKS);
      setElbowRotation(CARGO_LEVEL_THREE_ANGLE);
      break;
    case hatchFloorPickup:
      setArmTicks(HATCH_FLOOR_TICKS);
      setElbowRotation(HATCH_FLOOR_ANGLE);
      break;
    case hatchLevelOne:
      setArmTicks(HATCH_LEVEL_ONE_TICKS);
      setElbowRotation(HATCH_LEVEL_ONE_ANGLE);
      break;
    case hatchLevelTwo:
      setArmTicks(HATCH_LEVEL_TWO_TICKS);
      setElbowRotation(HATCH_LEVEL_TWO_ANGLE);
      break;
    case hatchLevelThree:
      setArmTicks(HATCH_LEVEL_THREE_TICKS);
      setElbowRotation(HATCH_LEVEL_THREE_ANGLE);
      break;
    default:
      break;
    }
  }

  public void setArmTicks(double encoderGoal) {
    error = getArmEncoder() - encoderGoal;
    errorChange = error - prevError;
    p = error * pk /* SmartDashboard.getNumber("pk ", 0) */;
    i += error * ik /* SmartDashboard.getNumber("ik ", 0) */;
    d = errorChange * dk /* SmartDashboard.getNumber("dk ", 0) */;
    power = p + i + d;
    setArmMotorSpeed(power);
    SmartDashboard.putNumber("ArmMotorPower", power);
    prevError = error;
    if (getBottomLimitSwitch()) { // Reset Encoder When Bottom Limit Switch is Pressed By Arm
      resetEncoder();
    }
  }

  public void setElbowRotation(double angleGoal) {
    // NEEDS TO BE NOT EMPTY - Should be similar to shoulder setTicks
  }

  public void setArmMotorSpeed(double speed) {
    armMotorMaster.set(speed);
  }

  public double getArmEncoder() {
    return armMotorMaster.getSelectedSensorPosition();
  }

  public void resetEncoder() {
    armMotorMaster.setSelectedSensorPosition(0, 0, 10);
  }

  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }

  public double getShoulderSpeed() {
    return errorChange; // errorChange from the PID calculations should be equal to speed
  }

  public double getShoulderPower() {
    return power;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
