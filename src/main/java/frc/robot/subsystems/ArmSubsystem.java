package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.arm.ArmManualControl;

/**
 * The ArmSubsystem is responsible for the shoulder and elbow motor control.
 * 
 * The shoulder motor moves the arm up and down.
 * 
 * The elbow motor moves the manipulator up and down.
 */
public class ArmSubsystem extends Subsystem {

  private double prevError, error, errorChange, elbowPower, shoulderPower, p, i, d;
  private final double shoulderPK = 0, shoulderIK = 0, shoulderDK = 0;// TEMP
  private final double elbowPK = 0, elbowIK = 0, elbowDK = 0;// TEMP
  private final double manualShoulderK = 0; // TEMP
  private final double CARGO_FLOOR_SHOULDER = 0.1;// TEMP
  private final double CARGO_PLAYER_STATION_SHOULDER = 0.2;// TEMP
  private final double CARGO_LEVEL_ONE_SHOULDER = 0.142;
  private final double CARGO_LEVEL_TWO_SHOULDER = 0.543;
  private final double CARGO_LEVEL_THREE_SHOULDER = 0.922;
  private final double HATCH_FLOOR_SHOULDER = 0.6; // TEMP
  private final double HATCH_LEVEL_ONE_SHOULDER = 0.07266;
  private final double HATCH_LEVEL_TWO_SHOULDER = 0.495;
  private final double HATCH_LEVEL_THREE_SHOULDER = 0.8975;
  private final double DEFAULT_STATE_ELBOW = 3.2;
  private final double STRAIGHT_FORWARDS_ELBOW = 2.35;
  private final double PARALLEL_TO_FLOOR_ELBOW = 0.23;

  private static WPI_TalonSRX shoulderMotor = new WPI_TalonSRX(RobotMap.rightShoulderMotor); // TEMP PORT NUMBER
  public static WPI_TalonSRX elbowMotor = new WPI_TalonSRX(RobotMap.elbowMotor); // TEMP PORT NUMBER

  DigitalInput bottomLimitSwitch = new DigitalInput(RobotMap.bottomShoulderLimitSwitch); // TEMP PORT NUMBER

  AnalogInput shoulderPotentiometer = new AnalogInput(RobotMap.shoulderPotentiometer);
  AnalogInput elbowPotentiometer = new AnalogInput(RobotMap.elbowPotentiometer);

  // TEMP CONSTANTS BELOW
  private static final int PEAK_CURRENT_AMPS = 55; /* threshold to trigger current limit */
  private static final int PEAK_TIME_MS = 0; /* how long after Peak current to trigger current limit */
  private static final int CONTIN_CURRENT_AMPS = 39; /* hold current after limit is triggered */
  private static final double OPEN_LOOP_RAMP_SECONDS = 0.1;
  // TEMP CONSTANTS ABOVE

  private static final double MAX_SHOULDER_VOLTAGE = 3.5;
  private static final double MIN_SHOULDER_VOLTAGE = 1.05;

  private static final double MAX_ELBOW_VOLTAGE = 4.5;
  private static final double MIN_ELBOW_VOLTAGE = 0.1;

  private final double UPWARDS_SHOULDER_LIMITER = 1; // was at 0.65 // TEMP NEEDS TESTING
  private final double DOWNWARDS_SHOULDER_LIMITER = 0.2; // TEMP NEEDS TESTING

  private final double UPWARDS_ELBOW_LIMITER = 1;
  private final double DOWNWARDS_ELBOW_LIMITER = 0.5;

  // Default Constructor Called At Start of Code
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

    elbowMotor.configPeakCurrentLimit(15);
    elbowMotor.configPeakCurrentDuration(PEAK_TIME_MS); /* this is a necessary call to avoid errata. */
    elbowMotor.configContinuousCurrentLimit(11);
    elbowMotor.enableCurrentLimit(true); /* honor initial setting */
    elbowMotor.configOpenloopRamp(OPEN_LOOP_RAMP_SECONDS);

  }

  /** Enumerations used in CargoMode and HatchMode Commands */
  public enum Levels {
    cargoFloorPickup, cargoPlayerStation, cargoLevelOne, cargoLevelTwo, cargoLevelThree, hatchFloorPickup,
    hatchLevelOne, hatchLevelTwo, hatchLevelThree, manualControl;
  }

  public void setLevel(Levels in) {
    setLevel(in, 0);
  }

  /**
   * Set the Arm to Constant Encoder Levels Based on Levels Enumeration.
   * 
   * @param in The level to move the arm to.
   */
  public void setLevel(Levels in, double manualOverride) {
    // NEEDS TO IMPLEMENT SETTING ELBOW STATES ONCE SHOULDER IS TESTED AND TUNED
    // WITH PID
    switch (in) {
    case manualControl:
      // May be backwards
      setShoulderPosition(
          getShoulderPosition() + (manualShoulderK * OI.xboxManipControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL)));
      break;
    case cargoFloorPickup:
      setShoulderPosition(CARGO_FLOOR_SHOULDER);
      break;
    case cargoPlayerStation:
      setShoulderPosition(CARGO_PLAYER_STATION_SHOULDER);
      break;
    case cargoLevelOne:
      setShoulderPosition(CARGO_LEVEL_ONE_SHOULDER);
      break;
    case cargoLevelTwo:
      setShoulderPosition(CARGO_LEVEL_TWO_SHOULDER);
      break;
    case cargoLevelThree:
      setShoulderPosition(CARGO_LEVEL_THREE_SHOULDER);
      break;
    case hatchFloorPickup:
      setShoulderPosition(HATCH_FLOOR_SHOULDER);
      // setElbowPosition(PARALLEL_TO_FLOOR_ELBOW);
      break;
    case hatchLevelOne:
      setShoulderPosition(HATCH_LEVEL_ONE_SHOULDER);
      // setElbowPosition(STRAIGHT_FORWARDS_ELBOW);
      break;
    case hatchLevelTwo:
      setShoulderPosition(HATCH_LEVEL_TWO_SHOULDER);
      // setElbowPosition(STRAIGHT_FORWARDS_ELBOW);
      break;
    case hatchLevelThree:
      setShoulderPosition(HATCH_LEVEL_THREE_SHOULDER);
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
    SmartDashboard.putNumber("positionGoal", positionGoal);
    prevError = error;
  }

  public void setShoulderMotorSpeed(double power) {
    if (power > 0 && getShoulderPotentiometeterVoltage() < MAX_SHOULDER_VOLTAGE) {
      power = UPWARDS_SHOULDER_LIMITER * power;
    } else if (power < 0 && getShoulderPotentiometeterVoltage() > MIN_SHOULDER_VOLTAGE) {
      power = DOWNWARDS_SHOULDER_LIMITER * power;
    } else {
      power = 0;
    }
    shoulderMotor.set(power);
  }

  public double getShoulderPosition() {
    return (shoulderPotentiometer.getAverageVoltage() - MIN_SHOULDER_VOLTAGE)
        / (MAX_SHOULDER_VOLTAGE - MIN_SHOULDER_VOLTAGE);
  }

  public void setElbowPosition(double positionVoltage) {
    error = elbowPotentiometer.getAverageVoltage() - positionVoltage;
    // error = getElbowPosition() - positionGoal;
    errorChange = error - prevError;
    p = error * /* elbowPK */ SmartDashboard.getNumber("elbowPK ", 0);
    i += error * /* elbowIK */ SmartDashboard.getNumber("elbowIK ", 0);
    d = errorChange * /* elbowDK */ SmartDashboard.getNumber("elbowDK ", 0);
    elbowPower = p + i + d;
    setElbowMotorSpeed(elbowPower);
    SmartDashboard.putNumber("ElbowMotorPower", elbowPower);
    SmartDashboard.putNumber("Current Goal", positionVoltage);
    prevError = error;
  }

  // 1.53 middle potentiometer
  public void setElbowMotorSpeed(double power) {
    if (power > 0.05 /* && getElbowPotentiometeterVoltage() > MIN_ELBOW_VOLTAGE */) {
      power = /* DOWNWARDS_ELBOW_LIMITER **/ power;
    } else if (power < -0.05 && getElbowPotentiometeterVoltage() < MAX_ELBOW_VOLTAGE) {
      power = UPWARDS_ELBOW_LIMITER * power;
    } else if (getElbowPotentiometeterVoltage() < MAX_ELBOW_VOLTAGE) {
      power = 0;// -.1
    } else {
      power = 0;
    }
    elbowMotor.set(power);
  }

  public double getElbowPosition() {
    return (elbowPotentiometer.getAverageVoltage() - MIN_ELBOW_VOLTAGE) / (MAX_ELBOW_VOLTAGE - MIN_ELBOW_VOLTAGE);
  }

  /**
   * Get bottom limit switch state.
   * 
   * @return false if tiggered, true if not triggered.
   */
  public boolean getBottomShoulderLimitSwitch() {
    return bottomLimitSwitch.get();
  }

  /**
   * Displays Diagnostics on SmartDashboard.
   */
  public void displayDiagnostics() {
    SmartDashboard.putNumber("Shoulder Motor Power", shoulderMotor.get());
    SmartDashboard.putNumber("Shoulder Current", shoulderMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shoulder Potentiometer", shoulderPotentiometer.getAverageVoltage());
    SmartDashboard.putNumber("Shoulder Position", getShoulderPosition());
    SmartDashboard.putNumber("Elbow Potentiometer", getElbowPotentiometeterVoltage());
    SmartDashboard.putNumber("Elbow Power", elbowMotor.get());
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
    // setDefaultCommand(new ArmManualControl());
    setDefaultCommand(new ArmManualControl());
  }
}