package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Command to switch between low and high gear.
 */
public class Shifter extends InstantCommand {
  DoubleSolenoid.Value gear;

  public Shifter() {
    super();
    requires(Robot.driveSubsystem);
    gear = Value.kOff;

  }

  public Shifter(DoubleSolenoid.Value value) {
    super();
    requires(Robot.driveSubsystem);
    gear = value;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (gear == DoubleSolenoid.Value.kOff)
      Robot.driveSubsystem.shift();
    else {
      Robot.driveSubsystem.setShiftSolenoid(gear);
    }

  }

}
