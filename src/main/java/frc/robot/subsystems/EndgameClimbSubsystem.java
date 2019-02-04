/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class EndgameClimbSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private Spark leftClimbArm = new Spark(RobotMap.leftClimbArm);
  private Spark rightClimbArm = new Spark(RobotMap.rightClimbArm);
  private Spark climbLegMotors = new Spark(RobotMap.elevatorDriveWheels);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
