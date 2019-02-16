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
  Levels desiredLevelShoulder = Levels.resetState;
  ElbowStates desiredLevelElbow = ElbowStates.reset;

  public ArmHatchMode() {
    requires(Robot.armSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (OI.xboxManipControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL) > 0.1
        || OI.xboxManipControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL) < -0.1) {
      desiredLevelShoulder = Levels.manualControl;
    }
    if (OI.xboxManipControl.getRawAxis(OI.DRIVE_RIGHT_JOYSTICK_VERTICAL) > 0.1
        || OI.xboxManipControl.getRawAxis(OI.DRIVE_RIGHT_JOYSTICK_VERTICAL) < -0.1) {
      desiredLevelElbow = ElbowStates.manualControl;
    }
    if (OI.xboxManipControl.getRawButton(OI.FLOOR_PICKUP)) {
      // floor pickup
      desiredLevelShoulder = Levels.hatchFloorPickup;
      desiredLevelElbow = ElbowStates.parallel;
    } else if (OI.xboxManipControl.getRawButton(OI.CARGO_PLAYER_STATION_PICKUP)) {
      // position for collecting cargo from the human player station
      desiredLevelShoulder = Levels.hatchLevelOne;
      desiredLevelElbow = ElbowStates.perpendicular;
    } else if (OI.xboxManipControl.getRawButton(OI.LEVEL_ONE_SELECTOR)) {
      // level one
      desiredLevelShoulder = Levels.hatchLevelOne;
      desiredLevelElbow = ElbowStates.perpendicular;
    } else if (OI.xboxManipControl.getRawButton(OI.LEVEL_TWO_SELECTOR)) {
      // level two
      desiredLevelShoulder = Levels.hatchLevelTwo;
      desiredLevelElbow = ElbowStates.perpendicular;
    } else if (OI.xboxManipControl.getRawButton(OI.LEVEL_THREE_SELECTOR)) {
      // level three
      desiredLevelShoulder = Levels.hatchLevelThree;
      desiredLevelElbow = ElbowStates.perpendicular;
    } else if (OI.xboxDriveControl.getRawButton(OI.RESET_ARM)) {
      desiredLevelShoulder = Levels.resetState;
      desiredLevelElbow = ElbowStates.reset;
    } else {
      // If no condition matches, then the desiredLevel value is left at its previous
      // state. Note that its starting state is initialized at the top of this class
      // definition.
    }
    Robot.armSubsystem.setLevel(desiredLevelShoulder, desiredLevelElbow,
        -1 * OI.xboxManipControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL),
        OI.xboxManipControl.getRawAxis(OI.DRIVE_RIGHT_JOYSTICK_VERTICAL));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
