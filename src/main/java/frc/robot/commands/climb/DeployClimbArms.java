package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ManipulatorSubsystem;

/**
 * Initialize begins a timer which is checked along with the current on the arm
 * motors, the current check ends the command when the arms begins pushing on
 * the platform, the timer ensures that the initial voltage spike when the arm
 * motors are activated doesn't stop them prematurely.
 */
public class DeployClimbArms extends Command {
  private double startTime;
  private final double CLIMB_ARMS_SPEED = -1; // Negative Moves Arm Out
  private final double ELEVATOR_DROP_SPEED = -0.2; // TEMP - just needs to be enough to make it touch the ground

  public DeployClimbArms() {
    requires(Robot.endgameClimbSubsystem);
  }

  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
    Robot.manipulatorSubsystem.setModeSolenoidState(ManipulatorSubsystem.MODE_SOLENOID_HATCH);
  }

  @Override
  protected void execute() {
    Robot.endgameClimbSubsystem.setClimbArmSpeeds(CLIMB_ARMS_SPEED);
    Robot.endgameClimbSubsystem.setElevatorSpeed(ELEVATOR_DROP_SPEED);
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
