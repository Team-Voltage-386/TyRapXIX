package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Runs the elevator in reverse in order to retract until a certain number of
 * limit switch changes occur.
 */
public class LevelThreeLiftElevator extends Command {

  private boolean currentState, prevState;
  private double limitSwitchChanges;

  public LevelThreeLiftElevator() {
    requires(Robot.endgameClimbSubsystem);
  }

  @Override
  protected void initialize() {
    limitSwitchChanges = 0;
  }

  @Override
  protected void execute() {
    // Counts limit switch changes
    currentState = Robot.endgameClimbSubsystem.getElevatorLimitSwitch();
    if (currentState != prevState) {
      limitSwitchChanges++;
    }

    Robot.endgameClimbSubsystem.setElevatorSpeed(1);

    prevState = currentState;
  }

  @Override
  protected boolean isFinished() {
    return limitSwitchChanges > 2 && !Robot.endgameClimbSubsystem.getElevatorLimitSwitch();
  }

  @Override
  protected void end() {
    Robot.endgameClimbSubsystem.setElevatorSpeed(0);
  }

  @Override
  protected void interrupted() {
  }
}
