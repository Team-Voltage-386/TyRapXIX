/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private Encoder armEncoder = new Encoder(RobotMap.armEncoderChannelA, RobotMap.armEncoderChannelB);
  private WPI_TalonSRX primaryArmMotor = new WPI_TalonSRX(RobotMap.armMotorPrimary);
  private WPI_TalonSRX followerArmMotor = new WPI_TalonSRX(RobotMap.armMotorFollower);
  private DigitalInput topLimitSwitch = new DigitalInput(RobotMap.topLimitSwitchPort);
  private DigitalInput bottomLimitSwitch = new DigitalInput(RobotMap.bottomLimitSwitchPort);

  public ArmSubsystem(){
    followerArmMotor.follow(primaryArmMotor);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void resetEncoder(){
    armEncoder.reset();
  }

  public void setArmMotorSpeed(double speed){
    primaryArmMotor.set(speed);
  }  

  public double getArmEncoderValue(){
    return armEncoder.getDistance();
  }

  public boolean getTopLimitSwitch(){
    return topLimitSwitch.get();
  }

  public boolean getBottomLimitSwitch(){
    return bottomLimitSwitch.get();
  }

}
