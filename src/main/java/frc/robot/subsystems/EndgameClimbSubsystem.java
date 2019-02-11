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

  private Spark leftClimbArm = new Spark(RobotMap.leftClimbArm);
  private Spark rightClimbArm = new Spark(RobotMap.rightClimbArm);
  private Spark climbElevatorMotor = new Spark(RobotMap.rearElevatorMotor);
  private Spark climbElevatorWheels = new Spark(RobotMap.elevatorDriveWheels);

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setClimbArmSpeeds(double speed) {
    leftClimbArm.set(speed);
    rightClimbArm.set(speed);
  }

  public void setElevatorSpeed(double speed) {
    climbElevatorMotor.set(speed);
  }

  public void setElevatorWheelsSpeed(double speed) {
    climbElevatorWheels.set(speed);
  }

}