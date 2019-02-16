/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class ManualClimb extends Command {
  public ManualClimb() {
    requires(Robot.endgameClimbSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.endgameClimbSubsystem.setElevatorSpeed(OI.xboxManipControl.getRawAxis(OI.MANIP_LEFT_JOYSTICK_VERTICAL));
    Robot.endgameClimbSubsystem.setClimbArmSpeeds(OI.xboxManipControl.getRawAxis(OI.MANIP_RIGHT_JOYSTICK_VERTICAL));
    if (OI.xboxManipControl.getRawButton(8)) {
      Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(1);
    } else if (OI.xboxManipControl.getRawButton(7)) {
      Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(-1);
    } else {
      Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(0);
    }
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
