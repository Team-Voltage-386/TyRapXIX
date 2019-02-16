package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem.Levels;

/**
 * Command used to set mode for the Hatch
 */
public class ArmHatchMode extends Command {
  Levels desiredLevel = Levels.hatchLevelOne;

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
    if (OI.xboxManipControl.getRawAxis(OI.LEFT_JOYSTICK_VERTICAL) > 0.1
        || OI.xboxManipControl.getRawAxis(OI.LEFT_JOYSTICK_VERTICAL) < -0.1) {
      Robot.armSubsystem.setShoulderMotorSpeed(OI.xboxManipControl.getRawAxis(OI.LEFT_JOYSTICK_VERTICAL));
    } else if (OI.xboxManipControl.getRawButton(OI.FLOOR_PICKUP)) {
      // floor pickup
      desiredLevel = Levels.hatchFloorPickup;
    } else if (OI.xboxManipControl.getRawButton(OI.LEVEL_ONE_SELECTOR)) {
      // level one
      desiredLevel = Levels.hatchLevelOne;
    } else if (OI.xboxManipControl.getRawButton(OI.LEVEL_TWO_SELECTOR)) {
      // level two
      desiredLevel = Levels.hatchLevelTwo;
    } else if (OI.xboxManipControl.getRawButton(OI.LEVEL_THREE_SELECTOR)) {
      // level three
      desiredLevel = Levels.hatchLevelThree;
    } else {
      // If no condition matches, then the desiredLevel value is left at its previous
      // state. Note that its starting state is initialized at the top of this class
      // definition.
    }
    Robot.armSubsystem.setLevel(desiredLevel);
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
