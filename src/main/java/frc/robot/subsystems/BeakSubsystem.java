/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class BeakSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static DoubleSolenoid hatchCapture = new DoubleSolenoid(RobotMap.hatchCaptureOpen,
      RobotMap.hatchCaptureClosed);

  public BeakSubsystem() {
  }

  public enum HatchScoringStates {
    beakOpen, beakRelease;
  }

  public void setState(HatchScoringStates in) {
    switch (in) {
    case beakOpen:
      hatchCapture.set(DoubleSolenoid.Value.kForward);
      break;
    case beakRelease:
      hatchCapture.set(DoubleSolenoid.Value.kReverse);
      break;
    default:
      break;
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
