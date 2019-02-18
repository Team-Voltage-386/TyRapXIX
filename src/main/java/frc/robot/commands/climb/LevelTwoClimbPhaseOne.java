package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Runs the already deployed climb arms and the lowers the elevator until a
 * certain number of limit switch changes occur and the limit switch reads
 * false.
 */
public class LevelTwoClimbPhaseOne extends Command {

  private boolean currentState = false, prevState = false;
  private int limitSwitchChanges;
  private final double DEFAULT_ARM_SPEED = -0.8; // Moves Climb Arms Out
  private final double DEFAULT_ELEVATOR_SPEED = -0.8; // Moves Elevator Down

  public LevelTwoClimbPhaseOne() {
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
    Robot.endgameClimbSubsystem.setClimbArmSpeeds(DEFAULT_ARM_SPEED);
    Robot.endgameClimbSubsystem.setElevatorSpeed(DEFAULT_ELEVATOR_SPEED);
    prevState = currentState;
    // SmartDashboard.putNumber("LimitSwitchChanges", limitSwitchChanges);
  }

  @Override
  protected boolean isFinished() {
    return limitSwitchChanges > 0 && !Robot.endgameClimbSubsystem.getElevatorLimitSwitch();
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}