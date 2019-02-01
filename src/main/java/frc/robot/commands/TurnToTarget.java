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
    requires(Robot.visionProcessing);
    requires(Robot.driveSubsystem);
    requires(Robot.spikeSubsystem);
  }

  ArrayList<RotatedRect[]> pairs = new ArrayList<RotatedRect[]>();
  RotatedRect[] bestPair = new RotatedRect[2];
  double center;
  double prevError, error = 0, p, kp, d, kd, i, ki;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveSubsystem.resetEncoder();
    Robot.driveSubsystem.resetPigeon();
    pairs = Robot.visionProcessing.visionProcess();
    Robot.spikeSubsystem.lightSwitch();
    i = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    pairs = Robot.visionProcessing.visionProcess();
    if(pairs.size()!=0){
      bestPair = pairs.get(0);

      for(int i = 0 ; i<pairs.size() ; i++){
        if(pairs.get(i)[0].size.width*pairs.get(i)[0].size.height + pairs.get(i)[1].size.width*pairs.get(i)[1].size.height >
          (bestPair[0].size.width * bestPair[0].size.height + bestPair[1].size.width * bestPair[1].size.height)){
            bestPair = pairs.get(i);
        }
      }
      
      prevError = error;
      error = ((bestPair[0].center.x + bestPair[1].center.x)/2 - center) / bestPair[0].size.width;

      //kp = 0.065, ki = 0.09, kd = 0.0
      p = error * SmartDashboard.getNumber("kp",0);
      i += error * SmartDashboard.getNumber("ki",0);
      d = (error - prevError) * SmartDashboard.getNumber("kd",0);
      

      Robot.driveSubsystem.driveTank(OI.xboxControl.getRawAxis(RobotMap.leftControllerY) -p - d - i,OI.xboxControl.getRawAxis(RobotMap.leftControllerY) + p + d + i);

      SmartDashboard.putNumber("Error", error);
      SmartDashboard.putNumber("Center of Pair", (bestPair[0].center.x + bestPair[1].center.x)/2 );

    }else{
      Robot.driveSubsystem.driveTank(OI.xboxControl.getRawAxis(RobotMap.leftControllerY), OI.xboxControl.getRawAxis(RobotMap.leftControllerY));

      SmartDashboard.putNumber("Error", 0);
      SmartDashboard.putNumber("Center of Pair", -1);
      SmartDashboard.putNumber("Center of Mat", center);
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
