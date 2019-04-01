package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ElbowStates;
import frc.robot.subsystems.ArmSubsystem.Levels;

public class SetManipArm extends Command {
    public SetManipArm() {
        // DO NOT add any requires() statements
        requires(Robot.armSubsystem);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        Robot.armSubsystem.setShoulderPosition(ArmSubsystem.FINAL_CLIMB_SHOULDER);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }
}
