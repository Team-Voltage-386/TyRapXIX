/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.ArrayList;

import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.VisionProcessing;

public class TurnToTarget extends Command {
  public TurnToTarget() {
    requires(Robot.visionProcessing);
    requires(Robot.driveSubsystem);
    requires(Robot.spikeSubsystem);
  }

  ArrayList<RotatedRect[]> pairs = new ArrayList<RotatedRect[]>();
  RotatedRect[] bestPair;
  double prevError, error = 0, p, kp, d, kd, i, ki;
  double bestPairChange, center;
  int indx;
  boolean best;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putString("Don't", "Initialize");
    Robot.driveSubsystem.resetEncoder();
    Robot.driveSubsystem.resetPigeon();
    pairs = Robot.visionProcessing.visionProcess();
    Robot.spikeSubsystem.lightSwitch();

    center = (Robot.visionProcessing.base.width() / 2) - 20;

    if (pairs.size() > 0) {
      bestPair = pairs.get(0);
      for (int i = 0; i < pairs.size(); i++) {
        if (Math.abs(center - (VisionProcessing.getPairCenter(pairs.get(i)))) <= Math
            .abs(center - (VisionProcessing.getPairCenter(bestPair)))) {
          bestPair = pairs.get(i);
          best = true;
          indx = i;
        }
      }
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    SmartDashboard.putString("Don't", "Execute");
    pairs = Robot.visionProcessing.visionProcess();

    if (pairs.size() > 0 && best) {

      bestPairChange = Math
          .abs(VisionProcessing.getPairCenter(pairs.get(0)) - VisionProcessing.getPairCenter(bestPair));

      SmartDashboard.putNumber("Pair Indx", indx);
      SmartDashboard.putNumber("Number of pairs", pairs.size());

      for (int i = 0; i < pairs.size(); i++) {
        if (Math.abs(
            VisionProcessing.getPairCenter(pairs.get(i)) - VisionProcessing.getPairCenter(bestPair)) < bestPairChange) {
          indx = i;
          bestPairChange = Math
              .abs(VisionProcessing.getPairCenter(pairs.get(indx)) - VisionProcessing.getPairCenter(bestPair));
        }

      }

      try {
        bestPair = pairs.get(indx);
      } catch (Exception e) {
        SmartDashboard.putString("Exception:", e + "");
      }

      SmartDashboard.putNumber("Center of Screen", Robot.visionProcessing.base.width() / 2);

      prevError = error;
      error = (VisionProcessing.getPairCenter(bestPair) - Robot.visionProcessing.base.width() / 2);

      p = error * SmartDashboard.getNumber("kp", 0);
      i += error * SmartDashboard.getNumber("ki", 0);
      d = (error - prevError) * SmartDashboard.getNumber("kd", 0);

      Robot.driveSubsystem.driveTank(
          (-.8 * OI.xboxDriveControl.getRawAxis(RobotMap.driveLeftJoystickVertical) + p + d + i),
          -OI.xboxDriveControl.getRawAxis(RobotMap.driveLeftJoystickVertical) - p - d - i);

      SmartDashboard.putNumber("Error", error);
      SmartDashboard.putNumber("Center of Pair", VisionProcessing.getPairCenter(bestPair));

    } else {
      Robot.driveSubsystem.driveTank(-0.8 * OI.xboxDriveControl.getRawAxis(RobotMap.driveLeftJoystickVertical),
          -OI.xboxDriveControl.getRawAxis(RobotMap.driveLeftJoystickVertical));

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
