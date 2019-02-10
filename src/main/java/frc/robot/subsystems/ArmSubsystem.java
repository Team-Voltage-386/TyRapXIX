package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
  private WPI_TalonSRX armMotorMaster = new WPI_TalonSRX(RobotMap.leftShoulderMotor);
  private WPI_TalonSRX armMotorFollower = new WPI_TalonSRX(RobotMap.rightShoulderMotor);

  // Limit Switch Declarations
  private DigitalInput bottomLimitSwitch = new DigitalInput(RobotMap.bottomArmLimitSwitch);

  // Default Constructor Called At Start of Code
  public ArmSubsystem() {
    armMotorFollower.follow(armMotorMaster);
    prevError = 0;
    p = 0;
    i = 0;
    d = 0;
  }

  /** Enumerations used in CargoMode and HatchMode Commands */
  public enum Levels {
    cargoFloorPickup, cargoPlayerStation, cargoLevelOne, cargoLevelTwo, cargoLevelThree, hatchFloorPickup,
    hatchLevelOne, hatchLevelTwo, hatchLevelThree;
  }

  /**
   * Set the Arm to Constant Encoder Levels Based on Levels Enumeration.
   * 
   * @param in The level to move the arm to.
   */
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

  /**
   * Set Arm to Given Goal Using PID.
   * 
   * @param encocderGoal The target goal value.
   */
  public void setArmTicks(double encoderGoal) {
    /*
     * can someone explain why we dont move this to a command? (I know, but I want a
     * reference for future students)
     */
    error = getArmEncoder() - encoderGoal;
    errorChange = error - prevError;
    p = error * pk /* SmartDashboard.getNumber("pk ", 0) */;
    i += error * ik /* SmartDashboard.getNumber("ik ", 0) */;
    d = errorChange * dk /* SmartDashboard.getNumber("dk ", 0) */;
    speed = p + i + d;
    setArmMotorSpeed(speed);
    SmartDashboard.putNumber("ArmMotorSpeed", speed);
    prevError = error;
    if (getBottomLimitSwitch()) { // Reset Encoder When Bottom Limit Switch is Pressed By Arm
      resetEncoder();
    }
  }

  /**
   * Set the arm motor speed to the specified value.
   * 
   * @param speed Values are between 0.0 and 1.0.
   */
  public void setArmMotorSpeed(double speed) {
    armMotorMaster.set(speed);
  }

  /**
   * Get Current Talon Encoder Value
   * 
   * @return The current encoder value.
   */
  public double getArmEncoder() {
    return armMotorMaster.getSelectedSensorPosition();
  }

  /**
   * Reset Arm Encoder.
   */
  public void resetEncoder() {
    armMotorMaster.setSelectedSensorPosition(0, 0, 10);
  }

  /**
   * Get bottom limit switch state.
   * 
   * @return false if tiggered, true if not triggered.
   */
  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }

  // No Default Command for ArmSubsystem
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
