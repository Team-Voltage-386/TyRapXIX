package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Drives the elevator up into the robot at full speed for 0.25 seconds to
 * ensure the correct number of limit switch changes occur when the elevator is
 * lifted in the next command.
 */
public class LiftElevatorSeconds extends Command {

  private double startTime, liftDuration;
  private final double ELEVATOR_MOTOR_SPEED = 1;

  public LiftElevatorSeconds(double time) {
    requires(Robot.endgameClimbSubsystem);
    startTime = Timer.getFPGATimestamp();
    liftDuration = time;
  }

  @Override
  protected void initialize() {
    SmartDashboard.putString("Lift status", "lifting");
  }

  @Override
  protected void execute() {
    Robot.endgameClimbSubsystem.setElevatorSpeed(ELEVATOR_MOTOR_SPEED);
  }

  @Override
  protected boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > liftDuration;
  }

  @Override
  protected void end() {
    SmartDashboard.putString("Lift status", "ended");
  }

  @Override
  protected void interrupted() {
  }
}
