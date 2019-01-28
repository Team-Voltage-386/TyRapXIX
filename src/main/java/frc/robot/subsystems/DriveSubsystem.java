/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static WPI_TalonSRX frontLeft = new WPI_TalonSRX(RobotMap.frontLeft);
  private static WPI_TalonSRX frontRight = new WPI_TalonSRX(RobotMap.frontRight);
  private static VictorSPX slaveLeft = new VictorSPX(RobotMap.rearLeftFollower);
  private static VictorSPX slaveRight = new VictorSPX(RobotMap.rearRightFollower);

  private static DoubleSolenoid shifter = new DoubleSolenoid(RobotMap.shifterLow,RobotMap.shifterHigh);

  private static DifferentialDrive differentialDrive = new DifferentialDrive(frontLeft, frontRight);

  public DriveSubsystem(){
    slaveLeft.follow(frontLeft);
    slaveRight.follow(frontRight);
    shifter.set(DoubleSolenoid.Value.kForward);
  }

  public void driveTank(double leftSpeed, double rightSpeed){
    differentialDrive.tankDrive(leftSpeed, rightSpeed); 
  }

  public void driveArcade(double xSpeed, double zRotation){
    differentialDrive.arcadeDrive(xSpeed, zRotation);
  }
  public void shift(){
    if(shifter.get()==DoubleSolenoid.Value.kForward){
      shifter.set(DoubleSolenoid.Value.kReverse);
    }else{
      shifter.set(DoubleSolenoid.Value.kForward);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new TankDrive());
  }
}
