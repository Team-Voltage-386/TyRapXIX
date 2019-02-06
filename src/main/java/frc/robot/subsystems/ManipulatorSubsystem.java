/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
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

  DoubleSolenoid cargoSolenoid = new DoubleSolenoid(RobotMap.beakRetractOpen, RobotMap.beakRetractClosed);
  DoubleSolenoid hatchSolenoid = new DoubleSolenoid(RobotMap.hatchCaptureOpen, RobotMap.hatchCaptureClosed);

  Spark cargoIntake = new Spark(RobotMap.cargoRollerMotor);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // setDefaultCommand(new ManipulatorManualControl());
    setDefaultCommand(new ManipulatorHatchMode());
  }

  public void switchCargoSolenoidState() {
    if (cargoSolenoid.get() == DoubleSolenoid.Value.kForward) {
      cargoSolenoid.set(DoubleSolenoid.Value.kReverse);
    } else {
      cargoSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void switchHatchSolenoidState() {
    if (hatchSolenoid.get() == DoubleSolenoid.Value.kForward) {
      hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
    } else {
      hatchSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void setCargoSolenoidState(Value state) {
    cargoSolenoid.set(state);
  }

  public void setHatchSolenoidState(Value state) {
    hatchSolenoid.set(state);
  }

  public void setCargoIntakeSpeed(double speed) {
    cargoIntake.set(speed);
  }

}
