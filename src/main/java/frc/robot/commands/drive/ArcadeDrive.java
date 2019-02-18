package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

/**
 * Command used to run the driver controller in arcade drive. <em>Left Analog Y
 * Axis</em> goes forward, and <em>Right Analog X Axis</em> (Left/Right) turn
 * the robot.
 * 
 * Note that this command has a built in speed limiter.
 */
public class ArcadeDrive extends Command {

  public ArcadeDrive() {
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Called repeatedly when this Command is scheduled to run

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double xSpeed = OI.xboxDriveControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL);
    double zRotation = OI.xboxDriveControl.getRawAxis(OI.DRIVE_RIGHT_JOYSTICK_HORIZONTAL);
    Robot.driveSubsystem.driveArcade(xSpeed, -zRotation);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
