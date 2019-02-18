package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Runs the elevator in reverse in order to retract until a certain number of
 * limit switch changes occur.
 */
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
    SmartDashboard.putString("Ended", "NOT ENDED");
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

    SmartDashboard.putNumber("LimitSwitchChanges", limitSwitchChanges);
  }

  @Override
  protected boolean isFinished() {
    return limitSwitchChanges > 2 && !Robot.endgameClimbSubsystem.getElevatorLimitSwitch();
  }

  @Override
  protected void end() {
    Robot.endgameClimbSubsystem.setElevatorSpeed(0);
    SmartDashboard.putString("Ended", "END");
  }

  @Override
  protected void interrupted() {
    Robot.endgameClimbSubsystem.setElevatorSpeed(0);
  }
}
