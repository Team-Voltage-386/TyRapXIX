package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LevelTwoLiftElevator extends Command {

  private boolean currentState, prevState;
  private double limitSwitchChanges;

  public LevelTwoLiftElevator() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.endgameClimbSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    limitSwitchChanges = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    currentState = Robot.endgameClimbSubsystem.getElevatorLimitSwitch();
    if (currentState != prevState) {
      limitSwitchChanges++;
    }

    Robot.endgameClimbSubsystem.setElevatorSpeed(1);

    prevState = currentState;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return limitSwitchChanges > 0 && !Robot.endgameClimbSubsystem.getElevatorLimitSwitch();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.endgameClimbSubsystem.setElevatorSpeed(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
