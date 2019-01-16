/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

<<<<<<< HEAD
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

=======
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
  private static WPI_TalonSRX slaveLeft = new WPI_TalonSRX(RobotMap.slaveLeft);
  private static WPI_TalonSRX frontRight = new WPI_TalonSRX(RobotMap.frontRight);
  private static WPI_TalonSRX slaveRight = new WPI_TalonSRX(RobotMap.slaveRight);

  private static DoubleSolenoid shifter = new DoubleSolenoid(RobotMap.shifterPort1,RobotMap.shifterPort2);

  private static DifferentialDrive differentialDrive = new DifferentialDrive(frontLeft, frontRight);

  public DriveSubsystem(){
    slaveLeft.follow(frontLeft);
    slaveRight.follow(frontRight);
    shifter.set(DoubleSolenoid.Value.kForward);
  }

  public static void driveTank(double leftSpeed, double rightSpeed){
    differentialDrive.tankDrive(leftSpeed, rightSpeed); 
  }

  public static void shift(){
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
>>>>>>> master
}
