/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class RunAutoLevelThree extends InstantCommand {
  /**
   * Add your docs here.
   */
  public RunAutoLevelThree() {
    requires(Robot.driveSubsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (Robot.driveSubsystem.getUltrasonicDistance() > Robot.driveSubsystem.MINIMUM_CLEARANCE_DISTANCE) {
      /** Needs testing - solenoid states may be reversed */
      if (Robot.manipulatorSubsystem.getCargoSolenoidState() == Value.kForward) {
        new AutoScoringGroup(ArmSubsystem.Levels.cargoLevelThree).start();
      } else if (Robot.manipulatorSubsystem.getCargoSolenoidState() == Value.kReverse) {
        new AutoScoringGroup(ArmSubsystem.Levels.hatchLevelThree).start();
      }
    } else {
    }
  }

}
