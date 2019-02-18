package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class UltrasonicDriveElevatorWheels extends Command {

  private double distanceGoalInches;
  private final double CLIMB_ARMS_HOLDING_SPEED = -0.4; // Push Down to Hold Front
  private final double ELEVATOR_HOLDING_SPEED = -0.3; // Push Down to Hold Back
  private final double ELEVATOR_WHEELS_SPEED = -0.8; // Drives Forwards
  private final double DRIVE_WHEELS_SPEED = -0.6; // Drives Forwards

  public UltrasonicDriveElevatorWheels(double goal) {
    requires(Robot.endgameClimbSubsystem);
    distanceGoalInches = goal;
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.endgameClimbSubsystem.setClimbArmSpeeds(CLIMB_ARMS_HOLDING_SPEED);
    Robot.endgameClimbSubsystem.setElevatorSpeed(ELEVATOR_HOLDING_SPEED);
    Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(ELEVATOR_WHEELS_SPEED);
    Robot.driveSubsystem.driveTank(DRIVE_WHEELS_SPEED, DRIVE_WHEELS_SPEED);
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
