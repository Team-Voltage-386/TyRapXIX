/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Compressor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;




public class DriveSubsystem extends Subsystem {
  WPI_TalonSRX frontLeft = new WPI_TalonSRX(RobotMap.leftPrimaryDriveMotor);
  WPI_TalonSRX frontRight = new WPI_TalonSRX(RobotMap.rightPrimaryDriveMotor);

  /* extra talons for six motor drives */
  WPI_TalonSRX leftSlave1 = new WPI_TalonSRX(RobotMap.leftFollowerDriveMotor);
  WPI_TalonSRX rightSlave1 = new WPI_TalonSRX(RobotMap.rightFollowerDriveMotor);
  /*
   * Commented out because we removed // WPI_TalonSRX leftSlave2 = new
   * WPI_TalonSRX(RobotMap.leftFollowerDriveMotor2); // WPI_TalonSRX rightSlave2 =
   * new // WPI_TalonSRX(RobotMap.rightFollowerDriveMotor2);
   */

  DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);

  Compressor compressor = new Compressor(RobotMap.compressor);

  public void drive(){
    leftSlave1.follow(frontLeft);
    rightSlave1.follow(frontRight);
    compressor.start();
    
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
     * Drive using tank-style values: left speed, right speed.
     * 
     * @param ySpeed
     * 
     *                The left motor speed
     * @param y2Speed The right motor speed
     */
    public void driveTank(double ySpeed, double y2Speed) {
      drive.tankDrive(ySpeed, y2Speed);
  }

  public void resetEncoders() {
    //setSelectedSensorPosition(sensorPos, pidIdx, timeoutMs)
    frontLeft.setSelectedSensorPosition(0, 0, 10);
    frontRight.setSelectedSensorPosition(0, 0, 10);
  }
  
    /**
     * Stop the robot from moving.
     */
    public void stop() {
      frontLeft.set(0);
      frontRight.set(0);
  }
   /**
     * @return the left encoder value
     */
    public double getLeftEncoder() {
      return frontLeft.getSelectedSensorPosition(0);
  }

  /**
   * @return the right encoder value
   */
  public double getRightEncoder() {
      return frontRight.getSelectedSensorPosition(0);
  }

}
