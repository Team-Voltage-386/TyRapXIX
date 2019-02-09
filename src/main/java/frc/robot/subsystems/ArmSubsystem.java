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
import frc.robot.commands.ManualShoulder;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {

  private double prevError, error, errorChange, speed, p, i, d;
  private final double pk = 0.05, ik = 0.001, dk = 0.07;
  private final int CARGO_FLOOR_TICKS = 100;
  private final int CARGO_PLAYER_STATION_TICKS = 100;
  private final int CARGO_LEVEL_ONE_TICKS = 100;
  private final int CARGO_LEVEL_TWO_TICKS = 100;
  private final int CARGO_LEVEL_THREE_TICKS = 100;
  private final int HATCH_FLOOR_TICKS = 100;
  private final int HATCH_LEVEL_ONE_TICKS = 100;
  private final int HATCH_LEVEL_TWO_TICKS = 100;
  private final int HATCH_LEVEL_THREE_TICKS = 100;

  public static WPI_TalonSRX leftShoulder = new WPI_TalonSRX(RobotMap.leftShoulderMotor);
  public static WPI_TalonSRX rightShoulder = new WPI_TalonSRX(RobotMap.rightShoulderMotor);

  DigitalInput bottomLimitSwitch = new DigitalInput(RobotMap.bottomArmLimitSwitch);

  public ArmSubsystem() {
    rightShoulder.follow(leftShoulder);
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
      break;
    case cargoPlayerStation:
      setArmTicks(CARGO_PLAYER_STATION_TICKS);
      break;
    case cargoLevelOne:
      setArmTicks(CARGO_LEVEL_ONE_TICKS);
      break;
    case cargoLevelTwo:
      setArmTicks(CARGO_LEVEL_TWO_TICKS);
      break;
    case cargoLevelThree:
      setArmTicks(CARGO_LEVEL_THREE_TICKS);
      break;
    case hatchFloorPickup:
      setArmTicks(HATCH_FLOOR_TICKS);
      break;
    case hatchLevelOne:
      setArmTicks(HATCH_LEVEL_ONE_TICKS);
      break;
    case hatchLevelTwo:
      setArmTicks(HATCH_LEVEL_TWO_TICKS);
      break;
    case hatchLevelThree:
      setArmTicks(HATCH_LEVEL_THREE_TICKS);
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
    speed = p + i + d;
    setShoulderMotorSpeed(speed);
    SmartDashboard.putNumber("ArmMotorSpeed", speed);
    prevError = error;
    if (getBottomLimitSwitch()) { // Reset Encoder When Bottom Limit Switch is Pressed By Arm
      resetEncoder();
    }
  }

  public void setShoulderMotorSpeed(double speed) {
    leftShoulder.set(speed);
  }

  public double getArmEncoder() {
    return leftShoulder.getSelectedSensorPosition();
  }

  public void resetEncoder() {
    leftShoulder.setSelectedSensorPosition(0, 0, 10);
  }

  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualShoulder());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
