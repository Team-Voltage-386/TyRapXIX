package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.drive.Shifter;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class LevelTwoClimbGroup extends CommandGroup {

    public LevelTwoClimbGroup() {

        addSequential(new Shifter(DoubleSolenoid.Value.kReverse));
        addSequential(new DeployClimbArms());
        addSequential(new LevelTwoClimbPhaseOne());
        addParallel(new SetManipArm());
        addSequential(new UltrasonicDriveElevatorWheels(27));
        addSequential(new LiftElevatorSeconds(0.25));
        addParallel(new LevelTwoLiftElevator());
        addSequential(new FinalPhaseDrive(18));
        addSequential(new EndEverything());
    }
}
