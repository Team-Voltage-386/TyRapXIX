package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DeployClimbArms extends Command {

  private double startTime;

  public DeployClimbArms() {
    requires(Robot.endgameClimbSubsystem);
  }

  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  protected void execute() {
    Robot.endgameClimbSubsystem.setClimbArmSpeeds(-1); // Negative moves arms out
  }

  @Override
  protected boolean isFinished() {
    // PDP current statements ensure motors stop when pushing down on the platform
    return Timer.getFPGATimestamp() - startTime > 0.5 && Robot.endgameClimbSubsystem.getPDPCurrent(4) > 4
        && Robot.endgameClimbSubsystem.getPDPCurrent(11) > 4;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
