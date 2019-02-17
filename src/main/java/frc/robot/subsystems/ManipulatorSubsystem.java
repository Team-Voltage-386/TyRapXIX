package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.manipulator.ManipulatorHatchDefault;

/**
 * The ManipulatorSubsytem is responsible for controlling the beak and cargo
 * intake mechanisms.
 */
public class ManipulatorSubsystem extends Subsystem {

  // Constant Speeds
  private static final double CARGO_INTAKE_SPEED = 1;
  private static final double CARGO_OUTTAKE_SPEED = -1;
  private static final double DEFAULT_INTAKE_SPEED = .5;

  // Current Constants
  private static final int PEAK_CURRENT_AMPS = 35; /* threshold to trigger current limit */
  private static final int PEAK_TIME_MS = 0; /* how long after Peak current to trigger current limit */
  private static final int CONTIN_CURRENT_AMPS = 25; /* hold current after limit is triggered */
  private static final double OPEN_LOOP_RAMP_SECONDS = 0.1;

  // Solenoids
  DoubleSolenoid cargoSolenoid = new DoubleSolenoid(RobotMap.beakRetractOpen, RobotMap.beakRetractClosed);
  DoubleSolenoid hatchSolenoid = new DoubleSolenoid(RobotMap.hatchCaptureOpen, RobotMap.hatchCaptureClosed);

  // Talon Motors
  WPI_TalonSRX cargoIntakeMotor = new WPI_TalonSRX(RobotMap.cargoRollerMotor);

  // Limit Switches
  DigitalInput easyButton = new DigitalInput(RobotMap.easyButtonPort);

  public ManipulatorSubsystem() {

    // Current Limiting
    cargoIntakeMotor.configPeakCurrentLimit(PEAK_CURRENT_AMPS);
    cargoIntakeMotor.configPeakCurrentDuration(PEAK_TIME_MS); /* this is a necessary call to avoid errata. */
    cargoIntakeMotor.configContinuousCurrentLimit(CONTIN_CURRENT_AMPS);
    cargoIntakeMotor.enableCurrentLimit(true); /* honor initial setting */
    cargoIntakeMotor.configOpenloopRamp(OPEN_LOOP_RAMP_SECONDS);

  }

  /** Cargo State Enumerations used in ManipulatorMode Commands */
  public enum CargoIntakeDirection {
    cargoOut, cargoIn, cargoOff;
  }

  /**
   * Set cargo solenoid state.
   * 
   * @param state A DoubleSolenoid state
   */
  public void setCargoSolenoidState(Value state) {
    cargoSolenoid.set(state);
  }

  /**
   * Set hatch solenoid state.
   * 
   * @param state A DoubleSolenoid state
   */
  public void setHatchSolenoidState(Value state) {
    hatchSolenoid.set(state);
  }

  /**
   * Set cargo intake motor speed.
   * 
   * @param speed Value from 0.0 to 0.1
   */
  public void setCargoIntakeSpeed(double speed) {
    cargoIntakeMotor.set(speed);
  }

  /**
   * Set CargoIntake direction.
   * 
   * @param direction The CargoIntakeDirection value
   */
  public void setCargoIntakeDirection(CargoIntakeDirection direction) {
    if (direction == CargoIntakeDirection.cargoIn) {
      setCargoIntakeSpeed(CARGO_INTAKE_SPEED);
    } else if (direction == CargoIntakeDirection.cargoOut) {
      setCargoIntakeSpeed(CARGO_OUTTAKE_SPEED);
    } else if (direction == CargoIntakeDirection.cargoOff) {
      setCargoIntakeSpeed(DEFAULT_INTAKE_SPEED);
    }
  }

  public DoubleSolenoid.Value getHatchSolenoidState() {
    return hatchSolenoid.get();
  }

  public void displayDiagnostics() {
    SmartDashboard.putBoolean("easyButton", easyButton.get());
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManipulatorHatchDefault());
  }

}