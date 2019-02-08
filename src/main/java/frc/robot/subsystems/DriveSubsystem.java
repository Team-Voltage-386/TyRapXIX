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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Robot;
import frc.robot.commands.ArcadeDrive;

/*
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

  private static final int PEAK_CURRENT_AMPS = 35; /* threshold to trigger current limit */
  private static final int PEAK_TIME_MS = 0; /* how long after Peak current to trigger current limit */
  private static final int CONTIN_CURRENT_AMPS = 25; /* hold current after limit is triggered */

  public static final double OPEN_LOOP_RAMP_SECONDS = 0.1;
  private static final int NO_TIMEOUT = 0;

  public DriveSubsystem() {
    slaveLeft.follow(frontLeft);
    slaveRight.follow(frontRight);
    shifter.set(DoubleSolenoid.Value.kForward);
    frontLeft.setInverted(true);
    frontRight.setInverted(true);
    slaveLeft.setInverted(InvertType.FollowMaster);
    slaveRight.setInverted(InvertType.FollowMaster);

    frontLeft.configPeakCurrentLimit(PEAK_CURRENT_AMPS, ENCODER_TIMEOUT);
    frontLeft.configPeakCurrentDuration(PEAK_TIME_MS, ENCODER_TIMEOUT); /* this is a necessary call to avoid errata. */
    frontLeft.configContinuousCurrentLimit(CONTIN_CURRENT_AMPS, ENCODER_TIMEOUT);
    frontLeft.enableCurrentLimit(true); /* honor initial setting */

    frontRight.configPeakCurrentLimit(PEAK_CURRENT_AMPS, ENCODER_TIMEOUT);
    frontRight.configPeakCurrentDuration(PEAK_TIME_MS, ENCODER_TIMEOUT); /* this is a necessary call to avoid errata. */
    frontRight.configContinuousCurrentLimit(CONTIN_CURRENT_AMPS, ENCODER_TIMEOUT);
    frontRight.enableCurrentLimit(true); /* honor initial setting */

    frontRight.configOpenloopRamp(OPEN_LOOP_RAMP_SECONDS, NO_TIMEOUT);
    frontLeft.configOpenloopRamp(OPEN_LOOP_RAMP_SECONDS, NO_TIMEOUT);
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

  public double getUltrasonicVoltage() {
    return driveUltrasonic.getAverageVoltage();
  }

  public double getUltrasonicDistance() {
    return driveUltrasonic.getAverageVoltage(); // Needs to add distance conversion to inches frmo voltage
  }

  public double getPetentiometerVoltage() {
    return petentiometer.getAverageVoltage();
  }

}