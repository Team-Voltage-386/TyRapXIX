/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ManipulatorSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static DoubleSolenoid hatchCapture = new DoubleSolenoid(RobotMap.hatchCaptureOpen,
      RobotMap.hatchCaptureClosed);
  public static DoubleSolenoid beakRetract = new DoubleSolenoid(RobotMap.beakRetractOpen, RobotMap.beakRetractClosed);

  public static WPI_TalonSRX cargoIntake = new WPI_TalonSRX(RobotMap.cargoCapture);

  public static final double DEFAULT_SPEED = 0;

  public ManipulatorSubsystem() {
  }

  /** Used in CargoRelease to prevent any hiccups from previous button inputs */
  public void switchCargoSolenoidStateOpen() {
    if (beakRetract.get() == Value.kForward) {
      beakRetract.set(Value.kReverse);
    } else {
    }
  }

  /**
   * Used in HatchRelease to ensure the beak isn't spread before the beak closes
   * in case of accidental button press.
   */
  public void switchCargoSolenoidStateClosed() {
    if (beakRetract.get() == Value.kReverse) {
      beakRetract.set(Value.kForward);
    } else {
    }
  }

  /** Used to open beak only if not already open */
  public void switchHatchSolenoidStateOpen() {
    if (hatchCapture.get() == Value.kForward) {
      hatchCapture.set(Value.kReverse);
    } else {
    }
  }

  /** Used to close beak only if not already closed */
  public void switchHatchSolenoidStateClosed() {
    if (hatchCapture.get() == Value.kReverse) {
      hatchCapture.set(Value.kForward);
    } else {
    }
  }

  public void setCargoSolenoidState(Value state) {
    beakRetract.set(state);
  }

  public void setHatchSolenoidState(Value state) {
    hatchCapture.set(state);
  }

  public void setCargoIntakeSpeed(double speed) {
    cargoIntake.set(speed);
  }

  public void stop() {
    cargoIntake.set(DEFAULT_SPEED);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
