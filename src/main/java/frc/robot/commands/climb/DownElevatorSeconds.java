/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DownElevatorSeconds extends Command {

  private double startTime, downDuration;
  private final double ELEVATOR_MOTOR_SPEED = -1;

  public DownElevatorSeconds(double time) {
    requires(Robot.endgameClimbSubsystem);
    startTime = Timer.getFPGATimestamp();
    downDuration = time;
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.endgameClimbSubsystem.setElevatorSpeed(ELEVATOR_MOTOR_SPEED);
  }

  @Override
  protected boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > downDuration;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
