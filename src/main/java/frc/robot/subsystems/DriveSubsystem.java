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
 * The DriveSubsystem is the interface for controlling the drive train. It provides methods
 * for driving the motors and retrieving encoder values from those motors.
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

  public static AnalogInput driveUltrasonic = new AnalogInput(RobotMap.analogUltrasonic);

  /** threshold to trigger current limit */
  private static final int PEAK_CURRENT_AMPS = 35;
  /** how long after Peak current to trigger current limit */
  private static final int PEAK_TIME_MS = 0;
  /* hold current after limit is triggered */
  private static final int CONTIN_CURRENT_AMPS = 25;

  public int MINIMUM_CLEARANCE_DISTANCE = 25; // TEMP - inches from game elements bot must be before lifting arms

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
   * Set the speed of the left and right motor groups.
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
   * Set the speed and rotation of the robot.
   * 
   * @param xSpeed    Percent speed for forward/backward. 1.00 is 100% forward
   *                  speed while -1.00 is -100% forward (backward) speed
   * @param zRotation percent speed dictating how much it's turning left or right.
   *                  1.00 is 100% to the right and -1.00 is 100% to the left
   */
  public void driveArcade(double xSpeed, double zRotation) {
    differentialDrive.arcadeDrive(xSpeed, zRotation);
  }

  /**
   * Displays Diagnostics on SmartDashboard.
   */
  public void displayDiagnostics() {
    SmartDashboard.putNumber("Encoder Talon 1", getLeftEncoder());
    SmartDashboard.putNumber("Encoder Talon 3", getRightEncoder());
    SmartDashboard.putNumber("Yaw Degree", Robot.driveSubsystem.getPigeonYPR()[0]);
    SmartDashboard.putNumber("Pitch Degree", Robot.driveSubsystem.getPigeonYPR()[1]);
    SmartDashboard.putNumber("Roll Degree", Robot.driveSubsystem.getPigeonYPR()[2]);
  }

  /**
   * Shift gears.
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

  public double getUltrasonicVoltage() {
    return driveUltrasonic.getAverageVoltage();
  }

  /**
   * Returns the yaw/pitch/roll from the Pigeon.
   * 
   * @return The an array of doubles with yaw as value 0, pitch as value 1, and
   *         roll as value 2.
   */
  public double[] getPigeonYPR() {
    double[] ypr_deg = new double[3];
    pigeon.getYawPitchRoll(ypr_deg);
    return ypr_deg;
  }

  /**
   * Resets the Pigeon's yaw to 0.
   */
  public void resetPigeon() {
    pigeon.setYaw(0);
  }
}
