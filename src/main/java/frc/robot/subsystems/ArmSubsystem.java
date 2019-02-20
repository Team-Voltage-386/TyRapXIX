package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.arm.ArmHatchMode;

/**
 * The ArmSubsystem is responsible for the shoulder and elbow motor control.
 * 
 * The shoulder motor moves the arm up and down.
 * 
 * The elbow motor moves the manipulator up and down.
 */
public class ArmSubsystem extends Subsystem {

  // Current Limiting Constants
  private static final int PEAK_CURRENT_AMPS = 55; /* threshold to trigger current limit */
  private static final int PEAK_TIME_MS = 0; /* how long after Peak current to trigger current limit */
  private static final int CONTIN_CURRENT_AMPS = 39; /* hold current after limit is triggered */
  private static final double OPEN_LOOP_RAMP_SECONDS = 0.1;

  // Doubles for PID Calculations
  private double shoulderP = 0, elbowP = 0, elbowI = 0, elbowD = 0, elbowResetP = 0;
  private double prevError = 0, error = 0, errorChange = 0, elbowPower = 0, shoulderPower = 0;

  // Constants for Calculations
  private final double shoulderPK = -30;
  private final double elbowPK = 0.5, elbowIK = 0.015, elbowDK = 1.25, elbowResetPK = 1.4;

  // Position States
  private final double CARGO_FLOOR_SHOULDER = 0.0542;
  private final double CARGO_PLAYER_STATION_SHOULDER = 0.208;
  private final double CARGO_LEVEL_ONE_SHOULDER = 0.142;
  private final double CARGO_LEVEL_TWO_SHOULDER = 0.591;
  private final double CARGO_LEVEL_THREE_SHOULDER = 1.00;
  private final double HATCH_FLOOR_SHOULDER = 0.115;
  private final double HATCH_LEVEL_ONE_SHOULDER = 0.0437;
  private final double HATCH_LEVEL_TWO_SHOULDER = 0.495;
  private final double HATCH_LEVEL_THREE_SHOULDER = 0.943;
  private final double FINAL_CLIMB_SHOULDER = 0.158;
  private final double RESET_SHOULDER = 0.0;
  private final double RESET_ELBOW = 4.8;
  private final double PERPENDICULAR_ELBOW = 3.85;
  private final double PARALLEL_ELBOW = 1.57;
  private final double CARGO_FLOOR_ELBOW = 2.415;
  private static final double HUMAN_PLAYER_ELBOW = 3.5; // TEMP

  // Voltage Soft Limits
  private static final double MAX_SHOULDER_VOLTAGE = 3.5;
  private static final double MIN_SHOULDER_VOLTAGE = 1.05;
  private static final double MAX_ELBOW_VOLTAGE = 4.72;
  private static final double MIN_ELBOW_VOLTAGE = 1.4;

  // Speed Limiters for
  private final double UPWARDS_SHOULDER_LIMITER = 1;
  private final double DOWNWARDS_SHOULDER_LIMITER = 0.45; // tune in pit
  private final double UPWARDS_ELBOW_LIMITER = 1;
  private final double DOWNWARDS_ELBOW_LIMITER = 0.75;

  // Talon Motors
  private static WPI_TalonSRX shoulderMotor = new WPI_TalonSRX(RobotMap.rightShoulderMotor);
  public static WPI_TalonSRX elbowMotor = new WPI_TalonSRX(RobotMap.elbowMotor);

  // Potentiometer
  AnalogInput shoulderPotentiometer = new AnalogInput(RobotMap.shoulderPotentiometer);
  AnalogInput elbowPotentiometer = new AnalogInput(RobotMap.elbowPotentiometer);

  // Desired States for setLevel()
  private Levels desiredStateShoulder = Levels.resetState;
  private ElbowStates desiredStateElbow = ElbowStates.reset;

  // Default Constructor Called At Start of Code
  public ArmSubsystem() {

    // Current Limiting Shoulder
    shoulderMotor.configPeakCurrentLimit(PEAK_CURRENT_AMPS);
    shoulderMotor.configPeakCurrentDuration(PEAK_TIME_MS); /* this is a necessary call to avoid errata. */
    shoulderMotor.configContinuousCurrentLimit(CONTIN_CURRENT_AMPS);
    shoulderMotor.enableCurrentLimit(true); /* honor initial setting */
    shoulderMotor.configOpenloopRamp(OPEN_LOOP_RAMP_SECONDS);

    // Current Limiting Elbow
    elbowMotor.configPeakCurrentLimit(15);
    elbowMotor.configPeakCurrentDuration(PEAK_TIME_MS); /* this is a necessary call to avoid errata. */
    elbowMotor.configContinuousCurrentLimit(11);
    elbowMotor.enableCurrentLimit(true); /* honor initial setting */
    elbowMotor.configOpenloopRamp(OPEN_LOOP_RAMP_SECONDS);

  }

  /** Motor Power, Current, Potentiometers, Position */
  public void displayDiagnostics() {
    SmartDashboard.putNumber("Shoulder Raw Power", shoulderMotor.get());
    SmartDashboard.putNumber("Shoulder Current", shoulderMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shoulder Potentiometer", shoulderPotentiometer.getAverageVoltage());
    SmartDashboard.putNumber("Shoulder Position", getShoulderPosition());
    SmartDashboard.putNumber("Elbow Potentiometer", getElbowPotentiometerVoltage());
    SmartDashboard.putNumber("Elbow Raw Power", elbowMotor.get());
  }

  /** Shoulder Enumerations used in CargoMode and HatchMode Commands */
  public enum Levels {
    cargoFloorPickup, cargoPlayerStation, cargoLevelOne, cargoLevelTwo, cargoLevelThree, hatchFloorPickup,
    hatchLevelOne, hatchLevelTwo, hatchLevelThree, manualControl, resetState, finalClimb;
  }

  /** Elbow Enumerations used in CargoMode and HatchMode Commands */
  public enum ElbowStates {
    reset, parallel, perpendicular, humanPlayer, manualControl, elbowCargoFloorPickup;
  }

  /**
   * Set the Arm to Constant Encoder Levels Based on Levels Enumeration.
   * 
   * @param inShoulder The level to move the shoulder to.
   * @param inElbow    The level to move the elbow to.
   */
  public void setLevel(Levels inShoulder, ElbowStates inElbow, double manualOverrideShoulder,
      double manualOverrideElbow) {

    // Switch-Case Loop for Shoulder Position
    switch (inShoulder) {
    case manualControl:
      setShoulderMotorSpeed(manualOverrideShoulder);
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
      break;
    case hatchLevelOne:
      setShoulderPosition(HATCH_LEVEL_ONE_SHOULDER);
      break;
    case hatchLevelTwo:
      setShoulderPosition(HATCH_LEVEL_TWO_SHOULDER);
      break;
    case hatchLevelThree:
      setShoulderPosition(HATCH_LEVEL_THREE_SHOULDER);
      break;
    case finalClimb:
      setShoulderPosition(FINAL_CLIMB_SHOULDER);
      break;
    case resetState:
      // Prevents Elbow From Getting Caught on Bumper
      if (getElbowPotentiometerVoltage() > PERPENDICULAR_ELBOW - 0.1) {
        setShoulderPosition(RESET_SHOULDER);
      }
    default:
      break;
    }

    // Switch-Case Loops for Elbow
    switch (inElbow) {
    case manualControl:
      setElbowMotorSpeed(manualOverrideElbow);
      elbowI = 0;
      break;
    case reset:
      resetElbowPosition(RESET_ELBOW);
      break;
    case parallel:
      setElbowPosition(PARALLEL_ELBOW);
      break;
    case perpendicular:
      setElbowPosition(PERPENDICULAR_ELBOW);
      break;
    case elbowCargoFloorPickup:
      setElbowPosition(CARGO_FLOOR_ELBOW);
      break;
    case humanPlayer:
      setElbowPosition(HUMAN_PLAYER_ELBOW);
      break;
    default:
      break;
    }
  }

  /**
   * Use PID to move shoulder to given position.
   * 
   * @param positionGoal The goal position (percentage representation as a double
   *                     from 0.0 to 1.0)
   */
  public void setShoulderPosition(double positionGoal) {
    error = getShoulderPosition() - positionGoal;
    errorChange = error - prevError;
    if (error < 0)
      shoulderP = error * shoulderPK;
    else
      shoulderP = error * -10;
    shoulderPower = shoulderP;
    if (Math.abs(error) < 0.01) {
      shoulderPower = 0;
    }
    setShoulderMotorSpeed(shoulderPower);
    SmartDashboard.putNumber("ShoulderPositionGoal", positionGoal); // May be removed from master
    prevError = error;
  }

  /**
   * Use PID to move elbow to input position.
   * 
   * @param positionVoltage The target voltage represented as a decimal from 0.0
   *                        to 5.0
   */
  public void setElbowPosition(double positionVoltage) {
    error = elbowPotentiometer.getAverageVoltage() - positionVoltage;
    errorChange = error - prevError;
    elbowP = error * elbowPK;
    elbowI += error * elbowIK;
    elbowD = errorChange * elbowDK;
    elbowPower = elbowP + elbowI + elbowD;
    setElbowMotorSpeed(elbowPower);
    SmartDashboard.putNumber("ElbowCurrentGoal", positionVoltage); // May be removed from master
    prevError = error;
  }

  /**
   * Set shoulder to speed input, incorporating limiters.
   * 
   * @param power The requested power from 0.0 to 1.0
   */
  public void setShoulderMotorSpeed(double power) {

    // Ensures Power is Between -1 and 1
    if (power > 1) {
      power = 1;
    }
    if (power < -1) {
      power = -1;
    }

    // Speed Limiters by Direction and Max and Min Shoulder Positions
    if (power > 0 && getShoulderPotentiometerVoltage() < MAX_SHOULDER_VOLTAGE) {
      power = UPWARDS_SHOULDER_LIMITER * power;
    } else if (power < 0 && getShoulderPotentiometerVoltage() > MIN_SHOULDER_VOLTAGE) {
      power = DOWNWARDS_SHOULDER_LIMITER * power;
    } else {
      power = 0;
    }
    shoulderMotor.set(power);
  }

  /**
   * Set elbow to speed input, incorporating limiters.
   * 
   * @param power The requested power from 0.0 to 1.0
   */
  public void setElbowMotorSpeed(double power) {

    // Ensures Speed is Between -1 to 1
    if (power > 1) {
      power = 1;
    }
    if (power < -1) {
      power = -1;
    }

    // Speed Limiters by Direction and Max and Min Voltages
    if (power > 0.05 && getElbowPotentiometerVoltage() > MIN_ELBOW_VOLTAGE) {
      power = DOWNWARDS_ELBOW_LIMITER * power;
    } else if (power < -0.05 && getElbowPotentiometerVoltage() < MAX_ELBOW_VOLTAGE) {
      power = UPWARDS_ELBOW_LIMITER * power;
    } else {
      power = 0;
    }
    elbowMotor.set(power);
  }

  /**
   * Convert range between min and max voltages to a percentage position value.
   * 
   * @return The current shoulder position as a percentage represented from 0.0 to
   *         1.0
   */
  public double getShoulderPosition() {
    return (shoulderPotentiometer.getAverageVoltage() - MIN_SHOULDER_VOLTAGE)
        / (MAX_SHOULDER_VOLTAGE - MIN_SHOULDER_VOLTAGE);
  }

  // Analog Input 0 to 5
  public double getShoulderPotentiometerVoltage() {
    return shoulderPotentiometer.getAverageVoltage();
  }

  // Analog Input 0 to 5
  public double getElbowPotentiometerVoltage() {
    return elbowPotentiometer.getAverageVoltage();
  }

  // Access DesiredStateShoulder from ArmMode Commands
  public Levels getDesiredStateShoulder() {
    return desiredStateShoulder;
  }

  // Access DesiredStateElbow from ArmMode Commands
  public ElbowStates getDesiredStateElbow() {
    return desiredStateElbow;
  }

  // Set DesiredStateShoulder from ArmMode Commands
  public void setDesiredStateShoulder(Levels newDesiredGoal) {
    desiredStateShoulder = newDesiredGoal;
  }

  // Set DesiredStateElbow from ArmMode Commands
  public void setDesiredStateElbow(ElbowStates newDesiredGoal) {
    desiredStateElbow = newDesiredGoal;
    elbowI = 0;
  }

  /**
   * PID control specific for resetting the elbow position.
   * 
   * @param positionVoltage The targer position voltage
   */
  public void resetElbowPosition(double positionVoltage) {
    error = elbowPotentiometer.getAverageVoltage() - positionVoltage;
    elbowResetP = error * elbowResetPK;
    elbowPower = elbowResetP;
    setElbowMotorSpeed(elbowPower);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArmHatchMode());
  }
}