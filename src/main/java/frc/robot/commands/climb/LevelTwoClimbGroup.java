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
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    addSequential(new Shifter(DoubleSolenoid.Value.kReverse));
    addSequential(new DeployClimbArms());
    addSequential(new LevelTwoClimbPhaseOne());
    addSequential(new UltrasonicDriveElevatorWheels(27));
    addSequential(new LiftElevatorSeconds(0.25));
    addSequential(new LevelTwoLiftElevator(18));

  }
}
