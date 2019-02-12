/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.ArmManualControl;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {

  private double prevError, error, errorChange, elbowPower, shoulderPower, p, i, d;
  private final double shoulderPK = 0, shoulderIK = 0, shoulderDK = 0;
  private final double elbowPK = 0, elbowIK = 0, elbowDK = 0;
  private final int CARGO_FLOOR_TICKS = 100;
  private final int CARGO_PLAYER_STATION_TICKS = 100;
  private final int CARGO_LEVEL_ONE_TICKS = 100;
  private final int CARGO_LEVEL_TWO_TICKS = 100;
  private final int CARGO_LEVEL_THREE_TICKS = 100;
  private final int HATCH_FLOOR_TICKS = 100;
  private final int HATCH_LEVEL_ONE_TICKS = 100;
  private final int HATCH_LEVEL_TWO_TICKS = 100;
  private final int HATCH_LEVEL_THREE_TICKS = 100;

  private static WPI_TalonSRX shoulderMotor = new WPI_TalonSRX(RobotMap.rightShoulderMotor); // TEMP PORT NUMBER
  public static WPI_TalonSRX elbowMotor = new WPI_TalonSRX(RobotMap.elbowMotor); // TEMP PORT NUMBER

  DigitalInput bottomLimitSwitch = new DigitalInput(RobotMap.bottomArmLimitSwitch); // TEMP PORT NUMBER

  AnalogInput shoulderPotentiometer = new AnalogInput(RobotMap.shoulderPotentiometer);
  AnalogInput elbowPotentiometer = new AnalogInput(RobotMap.elbowPotentiometer);

  // TEMP CONSTANTS BELOW
  private static final int PEAK_CURRENT_AMPS = 35; /* threshold to trigger current limit */
  private static final int PEAK_TIME_MS = 0; /* how long after Peak current to trigger current limit */
  private static final int CONTIN_CURRENT_AMPS = 25; /* hold current after limit is triggered */
  private static final double OPEN_LOOP_RAMP_SECONDS = 0.1;
  // TEMP CONSTANTS ABOVE

  private static final double MAX_SHOULDER_VOLTAGE = 3.5;
  private static final double MIN_SHOULDER_VOLTAGE = 1.0;

  private static final double MAX_ELBOW_VOLTAGE = 2.65;
  private static final double MIN_ELBOW_VOLTAGE = 1.5;

  public ArmSubsystem() {

    prevError = 0;
    p = 0;
    i = 0;
    d = 0;

    shoulderMotor.configPeakCurrentLimit(PEAK_CURRENT_AMPS);
    shoulderMotor.configPeakCurrentDuration(PEAK_TIME_MS); /* this is a necessary call to avoid errata. */
    shoulderMotor.configContinuousCurrentLimit(CONTIN_CURRENT_AMPS);
    shoulderMotor.enableCurrentLimit(true); /* honor initial setting */
    shoulderMotor.configOpenloopRamp(OPEN_LOOP_RAMP_SECONDS);

    elbowMotor.configPeakCurrentLimit(PEAK_CURRENT_AMPS);
    elbowMotor.configPeakCurrentDuration(PEAK_TIME_MS); /* this is a necessary call to avoid errata. */
    elbowMotor.configContinuousCurrentLimit(CONTIN_CURRENT_AMPS);
    elbowMotor.enableCurrentLimit(true); /* honor initial setting */
    elbowMotor.configOpenloopRamp(OPEN_LOOP_RAMP_SECONDS);

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
      setShoulderPosition(CARGO_FLOOR_TICKS);
      break;
    case cargoPlayerStation:
      setShoulderPosition(CARGO_PLAYER_STATION_TICKS);
      break;
    case cargoLevelOne:
      setShoulderPosition(CARGO_LEVEL_ONE_TICKS);
      break;
    case cargoLevelTwo:
      setShoulderPosition(CARGO_LEVEL_TWO_TICKS);
      break;
    case cargoLevelThree:
      setShoulderPosition(CARGO_LEVEL_THREE_TICKS);
      break;
    case hatchFloorPickup:
      setShoulderPosition(HATCH_FLOOR_TICKS);
      break;
    case hatchLevelOne:
      setShoulderPosition(HATCH_LEVEL_ONE_TICKS);
      break;
    case hatchLevelTwo:
      setShoulderPosition(HATCH_LEVEL_TWO_TICKS);
      break;
    case hatchLevelThree:
      setShoulderPosition(HATCH_LEVEL_THREE_TICKS);
      break;
    default:
      break;
    }
  }

  public void setShoulderPosition(double positionGoal) {
    error = getShoulderPosition() - positionGoal;
    errorChange = error - prevError;
    p = error * shoulderPK /* SmartDashboard.getNumber("shoulderPK ", 0) */;
    i += error * shoulderIK /* SmartDashboard.getNumber("shoulderIK" ", 0) */;
    d = errorChange * shoulderDK /* SmartDashboard.getNumber("shoulderDK ", 0) */;
    shoulderPower = p + i + d;
    setShoulderMotorSpeed(shoulderPower);
    SmartDashboard.putNumber("ShoulderMotorSpeed", shoulderPower);
    prevError = error;
  }

  public void setShoulderMotorSpeed(double power) {
    if (power > 0 && getShoulderPotentiometeterVoltage() < MAX_SHOULDER_VOLTAGE) {
      power = 0.65 * power;
    } else if (power < 0 && getShoulderPotentiometeterVoltage() > MIN_SHOULDER_VOLTAGE) {
      power = 0.2 * power;
    } else {
      power = 0;
    }
    shoulderMotor.set(power);
  }

  public double getShoulderPosition() {
    return (shoulderPotentiometer.getAverageVoltage() - MIN_SHOULDER_VOLTAGE)
        / (MAX_SHOULDER_VOLTAGE - MIN_SHOULDER_VOLTAGE);
  }

  public void setElbowPosition(double positionGoal) {
    error = getElbowPosition() - positionGoal;
    errorChange = error - prevError;
    p = error * elbowPK /* SmartDashboard.getNumber("elbowPK ", 0) */;
    i += error * elbowIK /* SmartDashboard.getNumber("elbowIK ", 0) */;
    d = errorChange * elbowDK /* SmartDashboard.getNumber("elbowDK ", 0) */;
    elbowPower = p + i + d;
    setShoulderMotorSpeed(elbowPower);
    SmartDashboard.putNumber("ElbowMotorPower", elbowPower);
    prevError = error;
  }

  // 1.53 middle potentiometer
  public void setElbowMotorSpeed(double power) {
    if (power > 0 && getElbowPotentiometeterVoltage() < MAX_ELBOW_VOLTAGE) {
      power = 0.35 * power;
    } else if (power < 0 && getElbowPotentiometeterVoltage() > MIN_ELBOW_VOLTAGE) {
      power = 1 * power;
    } else {
      power = 0;
    }
    elbowMotor.set(power);
  }

  public double getElbowPosition() {
    return (elbowPotentiometer.getAverageVoltage() - MIN_ELBOW_VOLTAGE) / (MAX_ELBOW_VOLTAGE - MIN_ELBOW_VOLTAGE);
  }

  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }

  public double getShoulderPotentiometeterVoltage() {
    return shoulderPotentiometer.getAverageVoltage();
  }

  public double getElbowPotentiometeterVoltage() {
    return elbowPotentiometer.getAverageVoltage();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ArmManualControl());
  }
}