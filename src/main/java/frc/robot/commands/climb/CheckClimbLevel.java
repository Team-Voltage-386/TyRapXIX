package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Chooses Whether or not to Run ClimbGroup for Level Two or Three
 */
public class CheckClimbLevel extends InstantCommand {

  public CheckClimbLevel() {
    super();
    requires(Robot.endgameClimbSubsystem);
  }

  /**
   * Checks ultrasonic distance to determine which level group should be run, then
   * runs it. The ultrasonic is above the level 2 platform.
   */
  @Override
  protected void initialize() {
    if (Robot.driveSubsystem.getUltrasonicDistance() < 10) { // Temporary Number to Determine Which Climb to do
      new LevelThreeClimbGroup().start();
    } else {
      new LevelTwoClimbGroup().start();
    }
  }

}
