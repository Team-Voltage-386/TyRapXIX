// package frc.robot.commands.manipulator;

// import edu.wpi.first.wpilibj.command.Command;
// import frc.robot.OI;
// import frc.robot.Robot;
// import frc.robot.subsystems.ManipulatorSubsystem;

// public class ManipulatorHatchMode extends Command {

// public ManipulatorHatchMode() {
// requires(Robot.manipulatorSubsystem);
// }

// @Override
// protected void initialize() {
// // Closed HatchSolenoid
// Robot.manipulatorSubsystem.setHatchSolenoidState(ManipulatorSubsystem.HATCH_SOLENOID_CLOSED);
// // Not-Folded CargoSolenoid
// Robot.manipulatorSubsystem.setModeSolenoidState(ManipulatorSubsystem.MODE_SOLENOID_HATCH);
// // Ensure that Cargo Intake Stops
// Robot.manipulatorSubsystem.setCargoIntakeSpeed(0);
// }

// // Called repeatedly when this Command is scheduled to run
// @Override
// protected void execute() {
// if (OI.xboxManipControl.getRawButton(OI.INTAKE)) {
// // Opens HatchSolenoid
// Robot.manipulatorSubsystem.setHatchSolenoidState(ManipulatorSubsystem.HATCH_SOLENOID_OPENED);
// } else if (OI.xboxManipControl.getRawButton(OI.OUTAKE)) {
// // Closes HatchSolenoid
// Robot.manipulatorSubsystem.setHatchSolenoidState(ManipulatorSubsystem.HATCH_SOLENOID_CLOSED);
// }
// }

// @Override
// protected boolean isFinished() {
// return false;
// }

// @Override
// protected void end() {
// }

// // Called when another command which requires one or more of the same
// // subsystems is scheduled to run
// @Override
// protected void interrupted() {
// }
// }