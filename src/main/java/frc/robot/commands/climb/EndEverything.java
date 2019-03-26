
package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class EndEverything extends InstantCommand {

    public EndEverything() {
        super();
        requires(Robot.endgameClimbSubsystem);
        requires(Robot.driveSubsystem);
        requires(Robot.armSubsystem);
    }

    @Override
    protected void initialize() {
        Robot.endgameClimbSubsystem.setClimbArmSpeeds(0);
        Robot.endgameClimbSubsystem.setElevatorSpeed(0);
        Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(0);
        Robot.driveSubsystem.driveTank(0, 0);
        Robot.armSubsystem.setShoulderMotorSpeed(0);
    }

}
