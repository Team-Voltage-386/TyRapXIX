package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Runs the drive train and elevator wheels until it reaches a certain distance
 * from the alliance station wall.
 */

public class FinalPhaseDrive extends Command {

  double distanceGoalInches;

  public FinalPhaseDrive(double goal) {
    requires(Robot.endgameClimbSubsystem);
    requires(Robot.driveSubsystem);
    distanceGoalInches = goal;
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(-0.8);
    Robot.driveSubsystem.driveTank(-0.5, -0.5);
  }

  @Override
  protected boolean isFinished() {
    // Finishes when in the correct ultrasonic range
    return Robot.driveSubsystem.getUltrasonicDistance() < distanceGoalInches;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
