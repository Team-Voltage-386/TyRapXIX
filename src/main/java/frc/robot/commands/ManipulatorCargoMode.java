/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.ManipulatorSubsystem.CargoIntakeDirection;

public class ManipulatorCargoMode extends Command {

  CargoIntakeDirection cargoIntakeDirection = CargoIntakeDirection.cargoOff;

  public ManipulatorCargoMode() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.manipulatorSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.manipulatorSubsystem.setHatchSolenoidState(DoubleSolenoid.Value.kReverse); // TEMP MAYBE BACKWARDS
    Robot.manipulatorSubsystem.setCargoSolenoidState(DoubleSolenoid.Value.kReverse); // TEMP MAYBE BACKWARDS
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { // USES TEMPORARY JOYSTICK HORIZONTAL RIGHT
    if (OI.xboxManipControl.getRawButton(RobotMap.cargoPickup)) { // 7 TEMP PORT NUMBER MAYBE BACKWARDS
      cargoIntakeDirection = CargoIntakeDirection.cargoIn;
    } else if (OI.xboxManipControl.getRawButton(RobotMap.cargoRelease)) { // 8 TEMP PORT NUMBER MAYBE BACKWARDS
      cargoIntakeDirection = CargoIntakeDirection.cargoOut;
    } else {
      cargoIntakeDirection = CargoIntakeDirection.cargoOff;
    }
    Robot.manipulatorSubsystem.setCargoIntakeDirection(cargoIntakeDirection);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}