package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class LevelThreeClimbPhaseOne extends Command {

  private boolean currentState = false, prevState = false;
  private int limitSwitchChanges;
  private final double DEFAULT_ARM_SPEED = -1;
  private final double DEFAULT_ELEVATOR_SPEED = -1;

  public LevelThreeClimbPhaseOne() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.endgameClimbSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    limitSwitchChanges = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    currentState = Robot.endgameClimbSubsystem.getElevatorLimitSwitch();
    if (currentState != prevState) {
      limitSwitchChanges++;
    }
    Robot.endgameClimbSubsystem.setClimbArmSpeeds(DEFAULT_ARM_SPEED);
    Robot.endgameClimbSubsystem.setElevatorSpeed(DEFAULT_ELEVATOR_SPEED);
    prevState = currentState;
    SmartDashboard.putNumber("LimitSwitchChanges", limitSwitchChanges);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return limitSwitchChanges > 2 && !Robot.endgameClimbSubsystem.getElevatorLimitSwitch(); // TEMP MAY BE BACKWARDS
                                                                                            // DEPENDING ON LIMITSWITCH
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
