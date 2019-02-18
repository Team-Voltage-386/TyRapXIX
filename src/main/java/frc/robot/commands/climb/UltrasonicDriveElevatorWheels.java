package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Runs elevator wheels, climb arm wheels, and drive train once bot is lifted to
 * drive it onto the platform. Uses ultrasonic distance to ensure it only drives
 * far enough to stay on the platform.
 */

public class UltrasonicDriveElevatorWheels extends Command {

  private double distanceGoalInches;

  public UltrasonicDriveElevatorWheels(double goal) {
    requires(Robot.endgameClimbSubsystem);
    distanceGoalInches = goal;
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.endgameClimbSubsystem.setClimbArmSpeeds(-.4);
    Robot.endgameClimbSubsystem.setElevatorSpeed(-.3);
    Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(-0.8);
    Robot.driveSubsystem.driveTank(-0.6, -0.6);
  }

  @Override
  protected boolean isFinished() {
    return Robot.driveSubsystem.getUltrasonicDistance() < distanceGoalInches;
  }

  @Override
  protected void end() {
    Robot.driveSubsystem.driveTank(0, 0);
    Robot.endgameClimbSubsystem.setClimbArmSpeeds(0);
    Robot.endgameClimbSubsystem.setElevatorSpeed(0);
    Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(0);
  }

  @Override
  protected void interrupted() {
  }
}
