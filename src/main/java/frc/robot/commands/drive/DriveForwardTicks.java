package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class DriveForwardTicks extends Command {

  double speed, ticks;

  /** negative speed is forwards */
  public DriveForwardTicks(double inSpeed, double inTicks) {
    requires(Robot.cameraSubsystem);
    requires(Robot.driveSubsystem);
    speed = inSpeed;
    ticks = inTicks;
  }

  boolean isRun = false;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveSubsystem.setShiftSolenoid(DoubleSolenoid.Value.kReverse);
    Robot.cameraSubsystem.on();
    Robot.driveSubsystem.resetEncoders();
    SmartDashboard.putNumber("START ENCODER", Robot.driveSubsystem.getLeftEncoder());

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveSubsystem.driveTank(speed, speed);
    isRun = true;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(Robot.driveSubsystem.getLeftEncoder()) > ticks && isRun;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    isRun = false;
    Robot.driveSubsystem.driveTank(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}