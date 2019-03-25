package frc.robot.commands.drive;

import java.util.ArrayList;

import org.opencv.core.RotatedRect;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;

public class AutoGoToTarget extends Command {
  public AutoGoToTarget() {
    requires(Robot.cameraSubsystem);
    requires(Robot.driveSubsystem);
  }

  double prevError, error = 0, p, d, i;
  int indx;
  boolean best;
  double startTime;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
    Robot.driveSubsystem.resetEncoders();
    Robot.driveSubsystem.setShiftSolenoid(DoubleSolenoid.Value.kReverse);
    Robot.cameraSubsystem.on();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (Robot.pairCenter > 0) {
      prevError = error;
      error = (Robot.pairCenter - Robot.screenCenter);

      p = error * 0.0075;
      i += 0.0;
      d = (error - prevError) * .5;

      Robot.driveSubsystem.driveTank(-0.5 + p + d + i, -0.5 - p - d - i);

      SmartDashboard.putNumber("Error", error);
      SmartDashboard.putNumber("Center of Pair", Robot.pairCenter);

    } else {
      Robot.driveSubsystem.driveTank(-0.5, -0.5);

      SmartDashboard.putNumber("Error", 0);
      SmartDashboard.putNumber("Center of Pair", -1);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.driveSubsystem.getUltrasonicDistance() < 13;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}