package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.arm.ArmHatchMode;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.Shifter;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class LevelThreeClimbGroup extends CommandGroup {

    public LevelThreeClimbGroup() {

        addSequential(new Shifter(DoubleSolenoid.Value.kReverse));
        addSequential(new DeployClimbArms());
        addSequential(new LevelThreeClimbPhaseOne());
        addParallel(new SetManipArm());//
        addSequential(new UltrasonicDriveElevatorWheels(31));// 27
        addSequential(new LiftElevatorSeconds(0.75));
        addParallel(new LevelThreeLiftElevator());
        addSequential(new FinalPhaseDrive(18));
        // addSequential(new EndEverything());
        addParallel(new ManualClimb());
        addParallel(new ArmHatchMode());//
        addSequential(new ArcadeDrive());

    }
}