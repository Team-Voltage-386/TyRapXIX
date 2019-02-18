package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.drive.Shifter;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Command group that defines the command sequence of the endgame climb.
 */
public class LevelTwoClimbGroup extends CommandGroup {
  /**
   * Add your docs here.
   */

  public LevelTwoClimbGroup() {

    addSequential(new Shifter(DoubleSolenoid.Value.kReverse));
    addSequential(new DeployClimbArms());
    addSequential(new LevelTwoClimbPhaseOne());
    addSequential(new UltrasonicDriveElevatorWheels(27));
    addSequential(new LiftElevatorSeconds(0.25));
    addParallel(new LevelTwoLiftElevator());
    addSequential(new FinalPhaseDrive(18));
  }
}
