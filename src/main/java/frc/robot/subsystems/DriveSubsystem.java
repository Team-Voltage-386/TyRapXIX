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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.TankDrive;
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

  private static PigeonIMU pigeon = new PigeonIMU(RobotMap.pigeonPort);

  private static DifferentialDrive differentialDrive = new DifferentialDrive(frontLeft, frontRight);

  final int kPeakCurrentAmps = 35; /* threshold to trigger current limit */
  final int kPeakTimeMs = 0; /* how long after Peak current to trigger current limit */
  final int kContinCurrentAmps = 25; /* hold current after limit is triggered */

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
    resetEncoder();
    resetPigeon();

    frontLeft.configPeakCurrentLimit(kPeakCurrentAmps, 10);
    frontLeft.configPeakCurrentDuration(kPeakTimeMs, 10); /* this is a necessary call to avoid errata. */
    frontLeft.configContinuousCurrentLimit(kContinCurrentAmps, 10);
    frontLeft.enableCurrentLimit(true); /* honor initial setting */

    frontRight.configPeakCurrentLimit(kPeakCurrentAmps, 10);
    frontRight.configPeakCurrentDuration(kPeakTimeMs, 10); /* this is a necessary call to avoid errata. */
    frontRight.configContinuousCurrentLimit(kContinCurrentAmps, 10);
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

  public void straightDrive() {
    double p = (Robot.driveSubsystem.getPigeonYPR()[0]) * -0.01;
    Robot.driveSubsystem.driveTank(-.5 + p, -.5 - p);
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

  public void resetEncoder() {
    frontLeft.setSelectedSensorPosition(0, 0, 10);
    frontRight.setSelectedSensorPosition(0, 0, 10);
  }

  public void pTurn(int goal) {
    double p = (getPigeonYPR()[0] - goal) * -0.01;
    driveTank(p, -1 * p);
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
