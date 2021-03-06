package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

/**
 * setup used to run the driver controller in Tank Drive. In this mode, <em>Left
 * Analog Stick</em> controls the left motors on the robot, and <em>Right Analog
 * Stick</em> controls the right side of the robot
 */
public class TankDrive extends Command {
  public TankDrive() {
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveSubsystem.driveTank(OI.xboxDriveControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL),
        OI.xboxDriveControl.getRawAxis(OI.DRIVE_RIGHT_JOYSTICK_HORIZONTAL));
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
