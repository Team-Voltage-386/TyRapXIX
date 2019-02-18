package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LevelThreeLiftElevator extends Command {

  private boolean currentState, prevState;
  private double limitSwitchChanges;

  private final double ELEVATOR_MOTOR_SPEED = 1; // Moves Elevator Up

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

    Robot.endgameClimbSubsystem.setElevatorSpeed(ELEVATOR_MOTOR_SPEED);

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
