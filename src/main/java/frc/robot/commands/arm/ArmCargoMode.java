package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem.ElbowStates;
import frc.robot.subsystems.ArmSubsystem.Levels;

/**
 * This command will constantly check for a specific button to be pressed and
 * will set the mode for cargo accordingly.
 */
public class ArmCargoMode extends Command {

    public ArmCargoMode() {
        requires(Robot.armSubsystem);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        if (OI.xboxManipControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL) > 0.1
                || OI.xboxManipControl.getRawAxis(OI.DRIVE_LEFT_JOYSTICK_VERTICAL) < -0.1) {
            // Manual Override Shoulder
            Robot.armSubsystem.setDesiredStateShoulder(Levels.manualControl);
        }
        if (OI.xboxManipControl.getRawAxis(OI.DRIVE_RIGHT_JOYSTICK_VERTICAL) > 0.1
                || OI.xboxManipControl.getRawAxis(OI.DRIVE_RIGHT_JOYSTICK_VERTICAL) < -0.1) {
            // Manual Elbow Override
            Robot.armSubsystem.setDesiredStateElbow(ElbowStates.manualControl);
        }
        if (OI.xboxManipControl.getRawButton(OI.FLOOR_PICKUP)) {
            // Floor Pickup
            setLevels(Levels.cargoFloorPickup, ElbowStates.elbowCargoFloorPickup);
        } else if (OI.xboxManipControl.getRawButton(OI.CARGO_PLAYER_STATION_PICKUP)) {
            // Player Station
            setLevels(Levels.cargoPlayerStation, ElbowStates.humanPlayer);
        } else if (OI.xboxManipControl.getRawButton(OI.LEVEL_ONE_SELECTOR)) {
            // Level One
            setLevels(Levels.cargoLevelOne, ElbowStates.perpendicular);
        } else if (OI.xboxManipControl.getRawButton(OI.LEVEL_TWO_SELECTOR)) {
            // Level Two
            setLevels(Levels.cargoLevelTwo, ElbowStates.perpendicular);
        } else if (OI.xboxManipControl.getRawButton(OI.LEVEL_THREE_SELECTOR)) {
            // Level Three
            setLevels(Levels.cargoLevelThree, ElbowStates.perpendicular);
        } else if (OI.xboxManipControl.getRawButton(OI.RESET_ARM)) {
            // Reset
            setLevels(Levels.resetState, ElbowStates.reset);
        } else {
            // If no condition matches, then the desiredLevel value is left at its previous
            // state. Note that its starting state is initialized at the top of this class
            // definition.
        }
        Robot.armSubsystem.setLevel(Robot.armSubsystem.getDesiredStateShoulder(),
                Robot.armSubsystem.getDesiredStateElbow(),
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

    /** Set Shoulder and Elbow Desired Levels */
    private void setLevels(Levels shoulder, ElbowStates elbow) {
        Robot.armSubsystem.setDesiredStateShoulder(shoulder);
        Robot.armSubsystem.setDesiredStateElbow(elbow);
    }

}
