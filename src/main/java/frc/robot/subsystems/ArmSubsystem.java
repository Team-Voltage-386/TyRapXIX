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

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {

  private double prevError, error, errorChange, speed, p, i, d;
  private final double pk = 0.05, ik = 0.001, dk = 0.07;
  private final int CARGO_FLOOR_TICKS = -10;
  private final int CARGO_PLAYER_STATION_TICKS = -15;
  private final int CARGO_LEVEL_ONE_TICKS = -25;
  private final int CARGO_LEVEL_TWO_TICKS = -32;
  private final int CARGO_LEVEL_THREE_TICKS = -40;
  private final int HATCH_FLOOR_TICKS = -40;
  private final int HATCH_LEVEL_ONE_TICKS = -32;
  private final int HATCH_LEVEL_TWO_TICKS = -25;
  private final int HATCH_LEVEL_THREE_TICKS = -15;

  WPI_TalonSRX armMotorMaster = new WPI_TalonSRX(RobotMap.leftShoulderMotor);
  WPI_TalonSRX armMotorFollower = new WPI_TalonSRX(RobotMap.rightShoulderMotor);

  DigitalInput bottomLimitSwitch = new DigitalInput(RobotMap.bottomArmLimitSwitch);
  Boolean fwd_lmt = false;
  Boolean rev_lmt = false;

  public ArmSubsystem() {
    armMotorFollower.follow(armMotorMaster);
    prevError = 0;
    p = 0;
    i = 0;
    d = 0;
    // ** Commented code below copied from 3452 repo **/
    // elevator_1.configForwardSoftLimitThreshold(topSoftLimit, GZSRX.TIMEOUT
    // elevator_1.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX,
    // LimitSwitchNormal.NormallyOpen, elevator_2.getDeviceID(), 10)

    // ?? TBD there are two version of the config below. One needs the device ID and
    // the other doesn't. If we're querying the encoder to which the breakout board
    // is attached, do we use the one without the deviceID call?
    armMotorMaster.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX,
        LimitSwitchNormal.NormallyOpen, armMotorMaster.getDeviceID(), 10);
    armMotorMaster.configReverseLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, 10);
  }

  public enum Levels {
    cargoFloorPickup, cargoPlayerStation, cargoLevelOne, cargoLevelTwo, cargoLevelThree, hatchFloorPickup,
    hatchLevelOne, hatchLevelTwo, hatchLevelThree, defaultLevel;
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
    case defaultLevel:
      setArmTicks(0);
      break;
    default:
      break;
    }
  }

  public void setArmTicks(double encoderGoal) {
    SmartDashboard.putNumber("Current Encoder Goal", encoderGoal);
    error = getArmEncoder() - encoderGoal;
    errorChange = error - prevError;
    p = error * pk /* SmartDashboard.getNumber("pk ", 0) */;
    i += error * ik /* SmartDashboard.getNumber("ik ", 0) */;
    d = errorChange * dk /* SmartDashboard.getNumber("dk ", 0) */;
    speed = p + i + d;
    setArmMotorSpeed(speed);
    SmartDashboard.putNumber("ArmMotorSpeed", speed);
    prevError = error;

    // ** Commented code below copied from 3452 repo **/
    // mIO.elevator_fwd_lmt =
    // elevator_2.getSensorCollection().isFwdLimitSwitchClosed();
    // mIO.elevator_rev_lmt =
    // elevator_2.getSensorCollection().isRevLimitSwitchClosed();
    fwd_lmt = armMotorMaster.getSensorCollection().isFwdLimitSwitchClosed();
    rev_lmt = armMotorMaster.getSensorCollection().isRevLimitSwitchClosed();

    // TBD??? What exactly does configClearPositionOnLimit do? Does it simply allow
    // the encoder to be reset at some future time? Or will it automatically reset
    // the encoder if the limit is reached?
    armMotorMaster.configClearPositionOnLimitF(true, 10);
    // if bottom limit is hit, reset the encoder to zero. Maybe not necessary if the
    // above code does the reset for us?
    if (rev_lmt) {
      resetEncoder();
    }

    /*
     * if (!(getBottomLimitSwitch())) { // Reset Encoder When Bottom Limit Switch is
     * Pressed By Arm resetEncoder(); }
     */
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
    // mIO.elevator_fwd_lmt =
    // elevator_2.getSensorCollection().isFwdLimitSwitchClosed();
    return rev_lmt;
    // return bottomLimitSwitch.get();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // setDefaultCommand(new HatchMode());
  }
}
