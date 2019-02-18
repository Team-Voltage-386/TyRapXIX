package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LiftElevatorSeconds extends Command {

  private double startTime, liftDuration;

  public LiftElevatorSeconds(double time) {
    requires(Robot.endgameClimbSubsystem);
    startTime = Timer.getFPGATimestamp();
    liftDuration = time;
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.endgameClimbSubsystem.setElevatorSpeed(1);
  }

  @Override
  protected boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > liftDuration;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
