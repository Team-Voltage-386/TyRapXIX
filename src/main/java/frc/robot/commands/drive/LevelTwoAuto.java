/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.AutoResumeManualControl;
import frc.robot.commands.arm.AutoSetArmLevel;
import frc.robot.commands.manipulator.AutoHatchRelease;
import frc.robot.subsystems.ArmSubsystem.ElbowStates;
import frc.robot.subsystems.ArmSubsystem.Levels;

public class LevelTwoAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LevelTwoAuto() {
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

    addSequential(new DriveForwardTicks(-1, 900));
    addParallel(new AutoSetArmLevel(Levels.hatchLevelOne, ElbowStates.perpendicular));
    addSequential(new DriveForwardSeconds(0.75, 1.0));
    addSequential(new AutoGoToTarget());
    addSequential(new AutoHatchRelease());
    // addSequential(new DriveForwardSeconds(0.5, 2));
    addSequential(new AutoResumeManualControl());

  }
}