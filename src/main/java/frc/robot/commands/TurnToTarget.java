/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.ArrayList;

import org.opencv.core.RotatedRect;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class TurnToTarget extends Command {
  public TurnToTarget() {
    requires(Robot.cameraSubsystem);
    requires(Robot.driveSubsystem);
    requires(Robot.spikeSubsystem);
  }

  RotatedRect[] bestPair;
  double prevError, error = 0, p, d, i;
  double bestPairChange, center;
  int indx;
  boolean best;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveSubsystem.resetEncoders();
    Robot.driveSubsystem.resetPigeon();
    Robot.spikeSubsystem.on();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if(Robot.pairCenter > 0){
      prevError = error;
      error = (Robot.pairCenter - Robot.screenCenter);

      p = error * -0.001;
      i += error * -0;
      d = (error - prevError) * -0.005;

      Robot.driveSubsystem.driveTank(
          (OI.xboxDriveControl.getRawAxis(RobotMap.driveLeftJoystickVertical) + p + d + i),
          OI.xboxDriveControl.getRawAxis(RobotMap.driveLeftJoystickVertical) - p - d - i);

      SmartDashboard.putNumber("Error", error);
      SmartDashboard.putNumber("Center of Pair", Robot.pairCenter);

    } else {
      Robot.driveSubsystem.driveTank(OI.xboxDriveControl.getRawAxis(RobotMap.driveLeftJoystickVertical),
          OI.xboxDriveControl.getRawAxis(RobotMap.driveLeftJoystickVertical));

      SmartDashboard.putNumber("Error", 0);
      SmartDashboard.putNumber("Center of Pair", -1);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !OI.xboxDriveControl.getRawButton(6);
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