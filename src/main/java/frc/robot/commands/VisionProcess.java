/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionProcess extends Command {
  NetworkTableEntry error;
  double previousError;
  double finalError;
  double d = 0.0, i, p = 0.0;
  double kp = .02, ki = 0.0, kd = 0.0;
  NetworkTableInstance testInstance = NetworkTableInstance.getDefault();
  double defaultvalue;

  public VisionProcess() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.visionProcessing);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    testInstance.startServer();
    testInstance.setServerTeam(386, 1735);
    NetworkTable table = testInstance.getTable("datatable");

    error = table.getEntry("X");
    finalError = error.getDouble(defaultvalue);
    Robot.spikeSubsystem.lightSwitch();
    // error = Robot.visionProcessing.visionProcess();
    previousError = finalError;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    NetworkTable table = testInstance.getTable("datatable");

    error = table.getEntry("X");
    finalError = error.getDouble(defaultvalue);

    // error = Robot.visionProcessing.visionProcess();
    if (finalError == 0) {
      Robot.driveSubsystem.driveTank(0, 0);
    } else {
      p = finalError * kp;
      d = kd * (finalError - previousError);
      Robot.driveSubsystem.driveTank(p, -p);
    }

    // Robot.driveSubsystem.driveTank(-p, +p);
    SmartDashboard.putNumber("Final Error", finalError);
    SmartDashboard.putNumber("Error1", Robot.visionProcessing.error1);
    SmartDashboard.putNumber("Error2", Robot.visionProcessing.error2);
    SmartDashboard.putNumber("p", p);
    SmartDashboard.putNumber("d", d);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (!OI.xboxDriveControl.getRawButton(3));
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
