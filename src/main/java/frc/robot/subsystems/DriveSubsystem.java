/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static WPI_TalonSRX frontLeft = new WPI_TalonSRX(RobotMap.frontLeft);
  private static WPI_TalonSRX slaveLeft = new WPI_TalonSRX(RobotMap.slaveLeft);
  private static WPI_TalonSRX frontRight = new WPI_TalonSRX(RobotMap.frontRight);
  private static WPI_TalonSRX slaveRight = new WPI_TalonSRX(RobotMap.slaveRight);

  private static Encoder leftEncoder = new Encoder(1,0);

  private static DoubleSolenoid shifter = new DoubleSolenoid(RobotMap.shifterPort1,RobotMap.shifterPort2);

  private static PigeonIMU pigeon = new PigeonIMU(RobotMap.pigeonPort);

  private static DifferentialDrive differentialDrive = new DifferentialDrive(frontLeft, frontRight);

  public DriveSubsystem(){
    slaveLeft.follow(frontLeft);
    slaveRight.follow(frontRight);
    shifter.set(DoubleSolenoid.Value.kForward);
    resetEncoder();
    resetPigeon();
  }

  public void driveTank(double leftSpeed, double rightSpeed){
    SmartDashboard.putNumber("Left speed", leftSpeed);
    SmartDashboard.putNumber("Right speed", rightSpeed);
    SmartDashboard.putNumber("Gyro Yaw", Robot.driveSubsystem.getPigeonYPR()[0]);
    differentialDrive.tankDrive(leftSpeed, rightSpeed); 
  }

  public void straightDrive(){
    double p = (Robot.driveSubsystem.getPigeonYPR()[0]) * -0.01;
    Robot.driveSubsystem.driveTank(-.5+p,-.5-p); 
  }

  public void shift(){
    if(shifter.get()==DoubleSolenoid.Value.kForward){
      shifter.set(DoubleSolenoid.Value.kReverse);
    }else{
      shifter.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void resetEncoder(){
    frontLeft.setSelectedSensorPosition(0, 0, 10);
    frontRight.setSelectedSensorPosition(0, 0, 10);
  }

  public double getLeftEncoder() {
    return leftEncoder.get();
  }

  public double getRightEncoder() {
    return frontRight.getSelectedSensorPosition(0);
  }

  public double[] getPigeonYPR(){
    double[] ypr_deg = new double[3];
    pigeon.getYawPitchRoll(ypr_deg);
    return ypr_deg;
  }

  public void resetPigeon(){
    pigeon.setYaw(0);
  }

  public void pTurn(int goal){
    double p = (getPigeonYPR()[0] - goal) * -0.01;
    driveTank(p,-1*p);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new TankDrive());
  }
}
