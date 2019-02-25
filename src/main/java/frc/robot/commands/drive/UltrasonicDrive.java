// package frc.robot.commands.drive;

// import edu.wpi.first.wpilibj.command.Command;
// import frc.robot.Robot;

// public class UltrasonicDrive extends Command {

// private final double WHEEL_SPEED = 0.5; // TEMP constant needs to be tested
// (PID may be used later too)
// private double distanceGoalInches;

// public UltrasonicDrive(double goal) {
// // Use requires() here to declare subsystem dependencies
// // eg. requires(chassis);
// requires(Robot.driveSubsystem);
// distanceGoalInches = goal;
// }

// // Called just before this Command runs the first time
// @Override
// protected void initialize() {
// }

// // Called repeatedly when this Command is scheduled to run
// @Override
// protected void execute() {
// Robot.driveSubsystem.driveArcade(WHEEL_SPEED, 0);
// }

// // Make this return true when this Command no longer needs to run execute()
// @Override
// protected boolean isFinished() {
// return Robot.driveSubsystem.getUltrasonicDistance() < distanceGoalInches;
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
