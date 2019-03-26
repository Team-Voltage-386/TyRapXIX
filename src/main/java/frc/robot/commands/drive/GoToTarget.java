package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;

public class GoToTarget extends Command {
  public GoToTarget() {
    requires(Robot.cameraSubsystem);
    requires(Robot.driveSubsystem);
  }

  double prevError, error = 0, p, d, i;
  int indx;
  boolean best;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
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

      Robot.driveSubsystem.driveTank((OI.xboxDriveControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL) + p + d + i),
          OI.xboxDriveControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL) - p - d - i);

      SmartDashboard.putNumber("Error", error);
      SmartDashboard.putNumber("Center of Pair", Robot.pairCenter);

    } else {
      Robot.driveSubsystem.driveTank(OI.xboxDriveControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL),
          OI.xboxDriveControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL));

      SmartDashboard.putNumber("Error", 0);
      SmartDashboard.putNumber("Center of Pair", -1);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !OI.xboxDriveControl.getRawButton(1);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveSubsystem.resetEncoders();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}