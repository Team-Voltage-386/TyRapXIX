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

/*
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  private static WPI_TalonSRX frontLeft = new WPI_TalonSRX(RobotMap.frontLeft);
  private static WPI_TalonSRX frontRight = new WPI_TalonSRX(RobotMap.frontRight);
  private static WPI_TalonSRX slaveLeft = new WPI_TalonSRX(RobotMap.rearLeftFollower);
  private static WPI_VictorSPX slaveRight = new WPI_VictorSPX(RobotMap.rearRightFollower);

  public static int ENCODER_TIMEOUT = 10; // in milliseconds

  private static DoubleSolenoid shifter = new DoubleSolenoid(RobotMap.shifterLow, RobotMap.shifterHigh);

  private static DifferentialDrive differentialDrive = new DifferentialDrive(frontLeft, frontRight);

  private static PigeonIMU pigeon = new PigeonIMU(RobotMap.pigeonPort);
  /** threshold to trigger current limit */
  private static final int PEAK_CURRENT_AMPS = 35;
  /** how long after Peak current to trigger current limit */
  private static final int PEAK_TIME_MS = 0;
  /* hold current after limit is triggered */
  private static final int CONTIN_CURRENT_AMPS = 25;

  private static final double OPEN_LOOP_RAMP_SECONDS = 0.1; // 100 milliseconds

  public DriveSubsystem() {
    slaveLeft.follow(frontLeft);
    slaveRight.follow(frontRight);
    shifter.set(DoubleSolenoid.Value.kForward);
    // invert the lead and the follower motors
    frontLeft.setInverted(true);
    frontRight.setInverted(true);
    slaveLeft.setInverted(InvertType.FollowMaster);
    slaveRight.setInverted(InvertType.FollowMaster);

    frontLeft.configPeakCurrentLimit(PEAK_CURRENT_AMPS);
    frontLeft.configPeakCurrentDuration(PEAK_TIME_MS);
    frontLeft.configContinuousCurrentLimit(CONTIN_CURRENT_AMPS);
    frontLeft.enableCurrentLimit(true);

    frontRight.configPeakCurrentLimit(PEAK_CURRENT_AMPS);
    frontRight.configPeakCurrentDuration(PEAK_TIME_MS);
    frontRight.configContinuousCurrentLimit(CONTIN_CURRENT_AMPS);
    frontRight.enableCurrentLimit(true);

    frontRight.configOpenloopRamp(OPEN_LOOP_RAMP_SECONDS);
    frontLeft.configOpenloopRamp(OPEN_LOOP_RAMP_SECONDS);
  }

  /**
   * setup used to run the driver controller in Tank Drive. In this mode, <em>Left
   * Analog Stick</em> controls the left motors on the robot, and <em>Right Analog
   * Stick</em> controls the right side of the robot
   * 
   * @param leftSpeed  controls the percentage for the left motors to go
   *                   forward/backward. 1.00 is 100% forward speed on the left
   *                   motors, and -1.00 is 100% backwards
   * @param rightSpeed controls the percentage for the left motors to go
   *                   forward/backward. 1.00 is 100% forward speed for the right
   *                   motors, and -1.00 is 100% backwards
   */
  public void driveTank(double leftSpeed, double rightSpeed) {
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * setup used to run the driver controller in arcade drive. in this mode,
   * <em>Left Analog Y Axis</em> goes forward, and <em>Right Analog X Axis</em>
   * (Left/Right) turn the robot
   * 
   * @param xSpeed    Percent speed for forward/backward. 1.00 is 100% forward
   *                  speed while -1.00 is -100% forward (backward) speed
   * @param zRotation percent speed dictating how much it's turning left or right.
   *                  1.00 is 100% to the right and -1.00 is 100% to the left
   */
  public void driveArcade(double xSpeed, double zRotation) {
    differentialDrive.arcadeDrive(xSpeed, zRotation);
  }

  /** Displays Encoder Diagnostics on SmartDashboard */
  public void displayDiagnostics() {
    // display left and right encoder values on the dash during drive
    SmartDashboard.putNumber(Robot.ENCODER_TALON_1, getLeftEncoder());
    SmartDashboard.putNumber(Robot.ENCODER_TALON_3, getRightEncoder());
  }

  /**
   * <h2>The Gear Shift function</h2>
   * <p>
   * a toggle function that switches between high gear and low gear
   * </p>
   * <h3>High Gear</h3>
   * <p>
   * This is when the robot goes faster and the wheels roll freely, but it cant
   * turn without browning out
   * </p>
   * <h3>Low Gear</h3>
   * <p>
   * This is when the robot goes slower, but its more controllable. needs to be in
   * low gear to turn or move to exact positions easily
   * </p>
   */
  public void shift() {
    if (shifter.get() == DoubleSolenoid.Value.kForward) {
      shifter.set(DoubleSolenoid.Value.kReverse);
    } else {
      shifter.set(DoubleSolenoid.Value.kForward);
    }
  }

  /** Resets the Encoder tick values */
  public void resetEncoders() {
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
  }

  /** Gets the Left Encoders position in ticks */
  public double getLeftEncoder() {
    return frontLeft.getSelectedSensorPosition(RobotMap.ENCODER_PORT);
  }

  /** Gets the Right Encoder's position in ticks */
  public double getRightEncoder() {
    return frontRight.getSelectedSensorPosition(RobotMap.ENCODER_PORT);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArcadeDrive());
  }

  /** gets the yaw/pitch/roll from the Pigeon */
  public double[] getPigeonYPR() {
    double[] ypr_deg = new double[3];
    pigeon.getYawPitchRoll(ypr_deg);
    return ypr_deg;
  }

  /** Resets the Pigeons Yaw/Pitch/Roll. Technically resets the yaw to 0 */
  public void resetPigeon() {
    pigeon.setYaw(0);
  }
}
