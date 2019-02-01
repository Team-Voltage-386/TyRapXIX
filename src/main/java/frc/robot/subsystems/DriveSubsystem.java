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
import frc.robot.commands.ArcadeDrive;
import frc.robot.OI;

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

  public static final double DEFAULT_SPEED_MULTIPLIER = 0.75;
  public static final double BOOST_SPEED_MULTIPLIER = 1.0;
  public static final double FAST_AUTO_MODE_SPEED = 0.9;// .75
  public static final double SLOW_AUTO_MODE_SPEED = 0.5;
  public static double speedMultiplier = BOOST_SPEED_MULTIPLIER;
  public static final double DEAD_BAND_LIMIT = 0.0001;


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

  double xSpeed = OI.xboxDriveControl.getRawAxis(RobotMap.driveLeftJoystickVertical);
  double zRotation = OI.xboxDriveControl.getRawAxis(RobotMap.driveRightJoystickHorizontal);
  public void driveArcade(double xSpeed, double zRotation) {
    differentialDrive.arcadeDrive(adjustSpeed(xSpeed), deadBand(zRotation, DEAD_BAND_LIMIT));
  }

  public void shift(){
    if(shifter.get()==DoubleSolenoid.Value.kForward){
      shifter.set(DoubleSolenoid.Value.kReverse);
    }else{
      shifter.set(DoubleSolenoid.Value.kForward);
    }
  }

  
   /**
     * Boost the forward speed. Now used
     */
    public void startBoost() {
      speedMultiplier = BOOST_SPEED_MULTIPLIER;
  }

  /**
   * Stop boosting the forward speed. Not used
   */
  public void stopBoost() {
      speedMultiplier = DEFAULT_SPEED_MULTIPLIER;
  }

  /**
     * Applies adjustments to the speed (such as inverting the direction for
     * inverted motors, or applying a dead band).
     * 
     * @param speed The input speed
     * @return The adjusted speed
     */
    private double adjustSpeed(double speed) {
      return deadBand((-1 * BOOST_SPEED_MULTIPLIER * speed), DEAD_BAND_LIMIT);
  }

  private double deadBand(double in, double limit) {
    if (Math.abs(in) < limit) {
        return 0;
    } else {
        return in;
    }
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ArcadeDrive());
  }
}
