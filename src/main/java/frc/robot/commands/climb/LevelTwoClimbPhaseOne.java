package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Runs the already deployed climb arms and the lowers the elevator until a
 * certain number of limit switch changes occur and the limit switch reads
 * false.
 */
public class LevelTwoClimbPhaseOne extends Command {

    private final double DEFAULT_ARM_SPEED = -0.8; // Moves Climb Arms Out
    private final double DEFAULT_ELEVATOR_SPEED = -0.7; // Moves Elevator Down

    public LevelTwoClimbPhaseOne() {
        requires(Robot.endgameClimbSubsystem);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        Robot.endgameClimbSubsystem.setClimbArmSpeeds(DEFAULT_ARM_SPEED);
        Robot.endgameClimbSubsystem.setElevatorSpeed(DEFAULT_ELEVATOR_SPEED);
    }

    @Override
    protected boolean isFinished() {
        return !Robot.endgameClimbSubsystem.getElevatorLimitSwitch();
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }
}