package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * The ArmSubsystem is responsible for the shoulder and elbow motor control.
 * 
 * The shoulder motor moves the arm up and down.
 * 
 * The elbow motor moves the manipulator up and down.
 */
public class ArmSubsystem extends Subsystem {

  // Variable Initializations and Constant Declarations for Encoder Levels and PID
  // Constants
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

  // Talon Motor Declarations
  private WPI_TalonSRX shoulderMotor = new WPI_TalonSRX(RobotMap.leftShoulderMotor);
  private WPI_TalonSRX elbowMotor = new WPI_TalonSRX(RobotMap.elbowMotor);

  // Limit Switch Declarations
  private DigitalInput bottomShoulderLimitSwitch = new DigitalInput(RobotMap.bottomArmLimitSwitch);

  AnalogInput potentiometer = new AnalogInput(RobotMap.potentiometer);

  // TEMP CONSTANTS BELOW
  private static final int PEAK_CURRENT_AMPS = 35; /* threshold to trigger current limit */
  private static final int PEAK_TIME_MS = 0; /* how long after Peak current to trigger current limit */
  private static final int CONTIN_CURRENT_AMPS = 25; /* hold current after limit is triggered */
  private static final double OPEN_LOOP_RAMP_SECONDS = 0.1;
  // TEMP CONSTANTS ABOVE
  private static final double MAX_SHOULDER_VOLTAGE = 3.8; // TEMP
  private static final double MIN_SHOULDER_VOLTAGE = 1.5; // TEMP

  private final double UPWARDS_SHOULDER_LIMITER = 0.75; // TEMP NEEDS TESTING
  private final double DOWNWARDS_SHOULDER_LIMITER = 0.2; // TEMP NEEDS TESTING

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

    elbowMotor.configPeakCurrentLimit(PEAK_CURRENT_AMPS);
    elbowMotor.configPeakCurrentDuration(PEAK_TIME_MS); /* this is a necessary call to avoid errata. */
    elbowMotor.configContinuousCurrentLimit(CONTIN_CURRENT_AMPS);
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
    switch (in) {
    case manualControl:
      setShoulderMotorSpeed(manualOverride);
      break;
    case cargoFloorPickup:
      setShoulderTicks(CARGO_FLOOR_TICKS);
      break;
    case cargoPlayerStation:
      setShoulderTicks(CARGO_PLAYER_STATION_TICKS);
      break;
    case cargoLevelOne:
      setShoulderTicks(CARGO_LEVEL_ONE_TICKS);
      break;
    case cargoLevelTwo:
      setShoulderTicks(CARGO_LEVEL_TWO_TICKS);
      break;
    case cargoLevelThree:
      setShoulderTicks(CARGO_LEVEL_THREE_TICKS);
      break;
    case hatchFloorPickup:
      setShoulderTicks(HATCH_FLOOR_TICKS);
      break;
    case hatchLevelOne:
      setShoulderTicks(HATCH_LEVEL_ONE_TICKS);
      break;
    case hatchLevelTwo:
      setShoulderTicks(HATCH_LEVEL_TWO_TICKS);
      break;
    case hatchLevelThree:
      setShoulderTicks(HATCH_LEVEL_THREE_TICKS);
      break;
    default:
      break;
    }
  }

  /**
   * Set Arm to Given Goal Using PID.
   * 
   * The PID variables are handled inside this method implementation because
   * multiple commands are used to control the arm and set it to different
   * heights.
   * 
   * @param encocderGoal The target goal value.
   */
  public void setShoulderTicks(double encoderGoal) {
    error = getShoulderEncoder() - encoderGoal;
    errorChange = error - prevError;
    p = error * pk /* SmartDashboard.getNumber("pk ", 0) */;
    i += error * ik /* SmartDashboard.getNumber("ik ", 0) */;
    d = errorChange * dk /* SmartDashboard.getNumber("dk ", 0) */;
    speed = p + i + d;
    setShoulderMotorSpeed(speed);
    prevError = error;
    if (getBottomShoulderLimitSwitch()) { // Reset Encoder When Bottom Limit Switch is Pressed By Arm
      resetEncoder();
    }
  }

  /**
   * Set the arm motor speed to the specified value.
   * 
   * @param speed Values are between 0.0 and 1.0.
   */
  public void setShoulderMotorSpeed(double speed) {
    if (speed > 0 && getPotentiometeterVoltage() < MAX_SHOULDER_VOLTAGE) {
      speed = UPWARDS_SHOULDER_LIMITER * speed;
    } else if (speed < 0 && getPotentiometeterVoltage() > MIN_SHOULDER_VOLTAGE) {
      speed = DOWNWARDS_SHOULDER_LIMITER * speed;
    } else {
      speed = 0;
    }
    shoulderMotor.set(speed);
  }

  /**
   * Get Current Talon Encoder Value
   * 
   * @return The current encoder value.
   */
  public double getShoulderEncoder() {
    return shoulderMotor.getSelectedSensorPosition();
  }

  /**
   * Reset Arm Encoder.
   */
  public void resetEncoder() {
    shoulderMotor.setSelectedSensorPosition(0, 0, 10);
  }

  /**
   * Get bottom limit switch state.
   * 
   * @return false if tiggered, true if not triggered.
   */
  public boolean getBottomShoulderLimitSwitch() {
    return bottomShoulderLimitSwitch.get();
  }

  /**
   * Displays Diagnostics on SmartDashboard.
   */
  public void displayDiagnostics() {
    SmartDashboard.putNumber("Shoulder Motor Speed", shoulderMotor.get());
  }

  public double getPotentiometeterVoltage() {
    return potentiometer.getAverageVoltage();
  }

  // No Default Command for ArmSubsystem
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
