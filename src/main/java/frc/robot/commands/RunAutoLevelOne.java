/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.OI;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class RunAutoLevelOne extends InstantCommand {
  /**
   * Add your docs here.
   */
  public RunAutoLevelOne() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (Robot.manipulatorSubsystem.beakRetract.get() == Value.kForward
        && Robot.driveSubsystem.getUltrasonicDistance() > Robot.driveSubsystem.MINIMUM_CLEARANCE_DISTANCE) {

    } else if (Robot.manipulatorSubsystem.beakRetract.get() == Value.kReverse
        && Robot.driveSubsystem.getUltrasonicDistance() > Robot.driveSubsystem.MINIMUM_CLEARANCE_DISTANCE) {
    } else {
    }
  }

}
