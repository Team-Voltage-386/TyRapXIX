/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ManipulatorHatchMode;

/**
 * Add your docs here.
 */

public class ManipulatorSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // TEMP PORT NUMBER
  public DoubleSolenoid cargSolenoid = new DoubleSolenoid(RobotMap.beakRetractOpen, RobotMap.beakRetractClosed);
  // TEMP PORT NUMBER
  DoubleSolenoid hatchSolenoid = new DoubleSolenoid(RobotMap.hatchCaptureOpen, RobotMap.hatchCaptureClosed);

  WPI_TalonSRX cargoIntakeMotor = new WPI_TalonSRX(RobotMap.cargoRollerMotor); // TEMP PORT NUMBER

  // TEMP CONSTANTS BELOW
  private static final int PEAK_CURRENT_AMPS = 35; /* threshold to trigger current limit */
  private static final int PEAK_TIME_MS = 0; /* how long after Peak current to trigger current limit */
  private static final int CONTIN_CURRENT_AMPS = 25; /* hold current after limit is triggered */
  private static final double OPEN_LOOP_RAMP_SECONDS = 0.1;
  // TEMP CONSTANTS ABOVE
  private static final double CARGO_INTAKE_SPEED = 0.5;

  public ManipulatorSubsystem() {

    cargoIntakeMotor.configPeakCurrentLimit(PEAK_CURRENT_AMPS);
    cargoIntakeMotor.configPeakCurrentDuration(PEAK_TIME_MS); /* this is a necessary call to avoid errata. */
    cargoIntakeMotor.configContinuousCurrentLimit(CONTIN_CURRENT_AMPS);
    cargoIntakeMotor.enableCurrentLimit(true); /* honor initial setting */
    cargoIntakeMotor.configOpenloopRamp(OPEN_LOOP_RAMP_SECONDS);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // setDefaultCommand(new ManipulatorManualControl());
    setDefaultCommand(new ManipulatorHatchMode());
  }

  public enum CargoIntakeDirection {
    cargoOut, cargoIn, cargoOff;
  }

  // Not certain what location each state corresponds to
  public void switchCargoSolenoidState() {
    if (cargSolenoid.get() == DoubleSolenoid.Value.kForward) {
      cargSolenoid.set(DoubleSolenoid.Value.kReverse);
    } else {
      cargSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

}

  // Not Certain what location each state corresponds to
  public void switchHatchSolenoidState() {
    if (hatchSolenoid.get() == DoubleSolenoid.Value.kForward) {
      hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
    } else {
      hatchSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void setCargoSolenoidState(Value state) {
    cargSolenoid.set(state);
  }

  public void setHatchSolenoidState(Value state) {
    hatchSolenoid.set(state);
  }

  public void setCargoIntakeSpeed(double speed) {
    cargoIntakeMotor.set(speed);
  }

  public Value getCargoSolenoidState() {
    cargSolenoid.get();
  }

  public void setCargoIntakeDirection(CargoIntakeDirection direction) {
    if (direction == CargoIntakeDirection.cargoIn) {
      setCargoIntakeSpeed(CARGO_INTAKE_SPEED); // TEMP MAYBE BACKWARDS
    } else if (direction == CargoIntakeDirection.cargoOut) {
      setCargoIntakeSpeed(-CARGO_INTAKE_SPEED); // TEMP MAYBE BACKWARDS
    } else if (direction == CargoIntakeDirection.cargoOff) {
      setCargoIntakeSpeed(0);
    }
  }

}