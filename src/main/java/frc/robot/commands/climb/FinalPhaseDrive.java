package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class FinalPhaseDrive extends Command {

  double distanceGoalInches;

  public FinalPhaseDrive(double goal) {
    requires(Robot.endgameClimbSubsystem);
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
