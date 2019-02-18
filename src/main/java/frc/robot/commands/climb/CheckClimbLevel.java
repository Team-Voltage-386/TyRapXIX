package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Chooses Whether or not to Run ClimbGroup for Level Two or Three
 */
public class CheckClimbLevel extends InstantCommand {

  private final double ULTRASONIC_CHECKING_DISTANCE = 10;

  public CheckClimbLevel() {
    super();
    requires(Robot.endgameClimbSubsystem);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (Robot.driveSubsystem.getUltrasonicDistance() < ULTRASONIC_CHECKING_DISTANCE) { // Temporary Number to Determine
                                                                                       // Which Climb to do
      new LevelThreeClimbGroup().start();
    } else {
      new LevelTwoClimbGroup().start();
    }
  }

}
