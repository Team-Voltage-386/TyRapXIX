/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Robot;
import frc.robot.commands.ArcadeDrive;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static WPI_TalonSRX frontLeft = new WPI_TalonSRX(RobotMap.frontLeft);
  private static WPI_TalonSRX frontRight = new WPI_TalonSRX(RobotMap.frontRight);
  private static WPI_TalonSRX slaveLeft = new WPI_TalonSRX(RobotMap.rearLeftFollower);
  private static WPI_VictorSPX slaveRight = new WPI_VictorSPX(RobotMap.rearRightFollower);

  public static int ENCODER_TIMEOUT = 10; // in milliseconds

  private static DoubleSolenoid shifter = new DoubleSolenoid(RobotMap.shifterLow, RobotMap.shifterHigh);

  private static DifferentialDrive differentialDrive = new DifferentialDrive(frontLeft, frontRight);

  private static PigeonIMU pigeon = new PigeonIMU(RobotMap.pigeonPort);

  public DriveSubsystem() {
    slaveLeft.follow(frontLeft);
    slaveRight.follow(frontRight);
    shifter.set(DoubleSolenoid.Value.kForward);
    frontLeft.setInverted(true);
    frontRight.setInverted(true);
    slaveLeft.setInverted(InvertType.FollowMaster);
    slaveRight.setInverted(InvertType.FollowMaster);
  }

  public void driveTank(double leftSpeed, double rightSpeed) {
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void driveArcade(double xSpeed, double zRotation) {
    differentialDrive.arcadeDrive(xSpeed, zRotation);
  }

  public void displayDiagnostics() {
    SmartDashboard.putNumber(Robot.ENCODER_TALON_1, getLeftEncoder());
    SmartDashboard.putNumber(Robot.ENCODER_TALON_3, getRightEncoder());
  }

  public void shift() {
    if (shifter.get() == DoubleSolenoid.Value.kForward) {
      shifter.set(DoubleSolenoid.Value.kReverse);
    } else {
      shifter.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void resetEncoders() {
    frontLeft.setSelectedSensorPosition(RobotMap.ENCODER_PORT, 0, ENCODER_TIMEOUT);
    frontRight.setSelectedSensorPosition(RobotMap.ENCODER_PORT, 0, ENCODER_TIMEOUT);
  }

  public double getLeftEncoder() {
    return frontLeft.getSelectedSensorPosition(RobotMap.ENCODER_PORT);
  }

  public double getRightEncoder() {
    return frontRight.getSelectedSensorPosition(RobotMap.ENCODER_PORT);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ArcadeDrive());
  }

  public double[] getPigeonYPR() {
    double[] ypr_deg = new double[3];
    pigeon.getYawPitchRoll(ypr_deg);
    return ypr_deg;
  }

  public void resetPigeon() {
    pigeon.setYaw(0);
  }
}
