/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.climb.ManualClimb;

/**
 * Add your docs here.
 */
public class EndgameClimbSubsystem extends Subsystem { // CURRENT LIMITING ON CLIMB TALON

  private Spark leftClimbArm = new Spark(RobotMap.leftClimbArm);
  private Spark rightClimbArm = new Spark(RobotMap.rightClimbArm);
  private WPI_TalonSRX climbElevatorMotor = new WPI_TalonSRX(RobotMap.rearElevatorMotor);
  private Spark climbElevatorWheels = new Spark(RobotMap.elevatorDriveWheels);
  private DigitalInput elevatorLimitSwitch = new DigitalInput(RobotMap.elevatorLimitSwitch); // TEMP Port Number

  private PowerDistributionPanel pdp = new PowerDistributionPanel();

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ManualClimb());
  }

  public void setClimbArmSpeeds(double speed) {
    leftClimbArm.set(-1 * speed);
    rightClimbArm.set(speed);
  }

  public void setElevatorSpeed(double speed) {
    climbElevatorMotor.set(speed);
  }

  public void setElevatorWheelsSpeed(double speed) {
    climbElevatorWheels.set(speed);
  }

  public boolean getElevatorLimitSwitch() {
    return elevatorLimitSwitch.get();
  }

  public void displayDiagnostics() {
    SmartDashboard.putBoolean("ElevatorLimitSwitch", getElevatorLimitSwitch());
  }

  public double getPDPCurrent(int pdpPort) {
    return pdp.getCurrent(pdpPort);
  }

}
