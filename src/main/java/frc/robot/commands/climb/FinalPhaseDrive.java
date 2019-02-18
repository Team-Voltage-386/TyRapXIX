package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class FinalPhaseDrive extends Command {

  private double distanceGoalInches;
  private final double ELEVATOR_WHEELS_SPEED = -0.8; // Drives Forwards
  private final double DRIVE_WHEELS_SPEED = -0.5; // Drives Forwards

  public FinalPhaseDrive(double goal) {
    requires(Robot.endgameClimbSubsystem);
    distanceGoalInches = goal;
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(ELEVATOR_WHEELS_SPEED);
    Robot.driveSubsystem.driveTank(DRIVE_WHEELS_SPEED, DRIVE_WHEELS_SPEED);
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
