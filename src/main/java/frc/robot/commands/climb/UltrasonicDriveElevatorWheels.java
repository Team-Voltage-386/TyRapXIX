package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class UltrasonicDriveElevatorWheels extends Command {

  private double distanceGoalInches;

  public UltrasonicDriveElevatorWheels(double goal) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.endgameClimbSubsystem);
    distanceGoalInches = goal;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.endgameClimbSubsystem.setClimbArmSpeeds(-.4);
    Robot.endgameClimbSubsystem.setElevatorSpeed(-.3);
    Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(-0.8);
    Robot.driveSubsystem.driveTank(-0.6, -0.6);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.driveSubsystem.getUltrasonicDistance() < distanceGoalInches;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveSubsystem.driveTank(0, 0);
    Robot.endgameClimbSubsystem.setClimbArmSpeeds(0);
    Robot.endgameClimbSubsystem.setElevatorSpeed(0);
    Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
