// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved. */
// /* Open Source Software - may be modified and shared by FRC teams. The code
// */
// /* must be accompanied by the FIRST BSD license file in the root directory of
// */
// /* the project. */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands;

// import java.util.ArrayList;

// import org.opencv.core.RotatedRect;
// import org.opencv.imgproc.Imgproc;

// import edu.wpi.first.wpilibj.command.Command;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.OI;
// import frc.robot.Robot;
// import frc.robot.RobotMap;
// import frc.robot.subsystems.VisionProcessing;

// public class DriveToTarget extends Command {
// public DriveToTarget() {
// requires(Robot.visionProcessing);
// requires(Robot.driveSubsystem);
// // requires(Robot.spikeSubsystem);
// }

// ArrayList<RotatedRect[]> pairs = new ArrayList<RotatedRect[]>();
// RotatedRect[] bestPair;
// double prevError, error = 0, p, kp, d, kd, i, ki;
// double bestPairChange;
// int indx;
// boolean best;

// // Called just before this Command runs the first time
// @Override
// protected void initialize() {
// SmartDashboard.putString("Don't", "Initialize");
// Robot.driveSubsystem.resetEncoder();
// Robot.driveSubsystem.resetPigeon();
// pairs = Robot.visionProcessing.visionProcess();
// // Robot.spikeSubsystem.lightSwitch();
// i = 0;
// }

// // Called repeatedly when this Command is scheduled to run
// @Override
// protected void execute() {
// SmartDashboard.putString("Don't", "Execute");
// pairs = Robot.visionProcessing.visionProcess();

// if (pairs.size() > 0) {

// bestPair = new RotatedRect[2];
// bestPair = pairs.get(0);

// for (int i = 0; i < pairs.size(); i++) {
// if (Math.abs((Robot.visionProcessing.base.width() / 2) -
// (VisionProcessing.getPairCenter(pairs.get(i)))) <= Math
// .abs((Robot.visionProcessing.base.width() / 2) -
// (VisionProcessing.getPairCenter(bestPair)))) {
// bestPair = pairs.get(i);
// best = true;
// indx = i;
// }
// }

// SmartDashboard.putNumber("Pair Indx", indx);
// SmartDashboard.putNumber("Center of Screen",
// Robot.visionProcessing.base.width() / 2);

// prevError = error;
// error = (VisionProcessing.getPairCenter(bestPair) -
// Robot.visionProcessing.base.width() / 2);

// p = error * SmartDashboard.getNumber("kp", 0);
// i += error * SmartDashboard.getNumber("ki", 0);
// d = (error - prevError) * SmartDashboard.getNumber("kd", 0);

// // Robot.driveSubsystem.driveTank((-.8 * .75 + p + d + i), -.75 - p - d - i);

// SmartDashboard.putNumber("Error", error);
// SmartDashboard.putNumber("Center of Pair",
// VisionProcessing.getPairCenter(bestPair));

// } else {
// Robot.driveSubsystem.driveTank(0, 0);

// SmartDashboard.putNumber("Error", 0);
// SmartDashboard.putNumber("Center of Pair", -1);
// }

// }

// // Make this return true when this Command no longer needs to run execute()
// @Override
// protected boolean isFinished() {
// return false;
// }

// // Called once after isFinished returns true
// @Override
// protected void end() {
// }

// // Called when another command which requires one or more of the same
// // subsystems is scheduled to run
// @Override
// protected void interrupted() {
// }
// }
