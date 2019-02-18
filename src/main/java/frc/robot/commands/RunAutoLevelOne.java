/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem.ElbowStates;
import frc.robot.subsystems.ArmSubsystem.Levels;

/**
 * Add your docs here.
 */
public class RunAutoLevelOne extends InstantCommand {
  /**
   * Add your docs here.
   */
  public RunAutoLevelOne() {
    requires(Robot.driveSubsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (Robot.driveSubsystem.getUltrasonicVoltage() > Robot.driveSubsystem.MINIMUM_CLEARANCE_DISTANCE) {
      /** Needs testing - solenoid states may be reversed */
      if (Robot.manipulatorSubsystem.getCargoSolenoidState() == Value.kForward) {
        new AutoScoringGroup(Levels.cargoLevelOne, ElbowStates.parallel).start();
      } else if (Robot.manipulatorSubsystem.getCargoSolenoidState() == Value.kReverse) {
        new AutoScoringGroup(Levels.hatchLevelOne, ElbowStates.parallel).start();
      }
    } else {
    }
  }
}
