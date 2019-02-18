package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LevelTwoLiftElevator extends Command {

  private final double ELEVATOR_MOTOR_SPEED = 1; // Moves Elevator Up

  public LevelTwoLiftElevator() {
    requires(Robot.endgameClimbSubsystem);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    // Set elevator speed to retract
    Robot.endgameClimbSubsystem.setElevatorSpeed(ELEVATOR_MOTOR_SPEED);
  }

  @Override
  protected boolean isFinished() {
    // Picks up the limitswitch change and stops the elevator motor
    return !Robot.endgameClimbSubsystem.getElevatorLimitSwitch();
  }

  @Override
  protected void end() {
    Robot.endgameClimbSubsystem.setElevatorSpeed(0);
  }

  @Override
  protected void interrupted() {
  }
}
