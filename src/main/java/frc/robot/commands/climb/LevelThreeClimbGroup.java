package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.drive.Shifter;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class LevelThreeClimbGroup extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LevelThreeClimbGroup() {

    addSequential(new Shifter(DoubleSolenoid.Value.kReverse));
    addSequential(new DeployClimbArms());
    addSequential(new LevelThreeClimbPhaseOne());
    addSequential(new UltrasonicDriveElevatorWheels(27));
    addSequential(new LiftElevatorSeconds(0.25));
    addParallel(new LevelThreeLiftElevator());
    addSequential(new FinalPhaseDrive(18));
  }
}
