package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Command to switch between low and high gear.
 */
public class Shifter extends InstantCommand {
  public Shifter() {
    super();
    requires(Robot.driveSubsystem);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.driveSubsystem.shift();
  }

}
