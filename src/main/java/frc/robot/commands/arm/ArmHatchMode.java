package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem.ElbowStates;
import frc.robot.subsystems.ArmSubsystem.Levels;

/**
 * Command used to set mode for the Hatch
 */
public class ArmHatchMode extends Command {

  public ArmHatchMode() {
    requires(Robot.armSubsystem);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {

    if (OI.xboxManipControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL) > 0.1
        || OI.xboxManipControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL) < -0.1) {
      // Shoulder Manual Override
      Robot.armSubsystem.setDesiredStateShoulder(Levels.manualControl);
    }

    if (OI.xboxManipControl.getRawAxis(OI.DRIVE_RIGHT_JOYSTICK_VERTICAL) > 0.1
        || OI.xboxManipControl.getRawAxis(OI.DRIVE_RIGHT_JOYSTICK_VERTICAL) < -0.1) {
      // Elbow Manual Override
      Robot.armSubsystem.setDesiredStateElbow(ElbowStates.manualControl);
    }

    if (OI.xboxManipControl.getRawButton(OI.FLOOR_PICKUP)) {
      // Floor Pickup
      set(Levels.hatchFloorPickup, ElbowStates.parallel);
    } else if (OI.xboxManipControl.getRawButton(OI.CARGO_PLAYER_STATION_PICKUP)) {
      // Human Player Station
      set(Levels.hatchLevelOne, ElbowStates.perpendicular);
    } else if (OI.xboxManipControl.getRawButton(OI.LEVEL_ONE_SELECTOR)) {
      // Level One
      set(Levels.hatchLevelOne, ElbowStates.perpendicular);
    } else if (OI.xboxManipControl.getRawButton(OI.LEVEL_TWO_SELECTOR)) {
      // Level Two
      set(Levels.hatchLevelTwo, ElbowStates.perpendicular);
    } else if (OI.xboxManipControl.getRawButton(OI.LEVEL_THREE_SELECTOR)) {
      // Level Three
      set(Levels.hatchLevelThree, ElbowStates.perpendicular);
    } else if (OI.xboxManipControl.getRawButton(OI.RESET_ARM)) {
      // Reset
      set(Levels.resetState, ElbowStates.reset);
    } else {
      // If no condition matches, then the desiredLevel value is left at its previous
      // state. Note that its starting state is initialized at the top of this class
      // definition.
    }
    Robot.armSubsystem.setLevel(Robot.armSubsystem.getDesiredStateShoulder(), Robot.armSubsystem.getDesiredStateElbow(),
        -1 * OI.xboxManipControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL),
        OI.xboxManipControl.getRawAxis(OI.DRIVE_RIGHT_JOYSTICK_VERTICAL));
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  // Set Shoulder and Elbow Desired Levels
  private void set(Levels shoulder, ElbowStates elbow) {
    Robot.armSubsystem.setDesiredStateShoulder(shoulder);
    Robot.armSubsystem.setDesiredStateElbow(elbow);
  }

}