package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.ManipulatorSubsystem.CargoIntakeDirection;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ManipulatorCargoMode extends Command {

    // Enumeration to Feed Into setCargoIntakeDirection Method
    CargoIntakeDirection cargoIntakeDirection = CargoIntakeDirection.cargoDefault;

    public ManipulatorCargoMode() {
        requires(Robot.manipulatorSubsystem);
    }

    @Override
    protected void initialize() {
        // Closes HatchSolenoid for Ball Intake
        Robot.manipulatorSubsystem.setHatchSolenoidState(ManipulatorSubsystem.HATCH_SOLENOID_CLOSED);
        // Folds Back CargoSolenoid
        Robot.manipulatorSubsystem.setModeSolenoidState(ManipulatorSubsystem.MODE_SOLENOID_CARGO);
    }

    @Override
    protected void execute() {
        if (OI.xboxManipControl.getRawButton(OI.INTAKE)) {
            // Initiate Cargo Intake
            cargoIntakeDirection = CargoIntakeDirection.cargoIn;
        } else if (OI.xboxManipControl.getRawButton(OI.OUTAKE)) {
            // Initiate Cargo Outtake
            cargoIntakeDirection = CargoIntakeDirection.cargoOut;
        } else {
            // Ensures That Cargo Intake Resumes Default Intake Speed
            cargoIntakeDirection = CargoIntakeDirection.cargoDefault;
        }
        Robot.manipulatorSubsystem.setCargoIntakeDirection(cargoIntakeDirection);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}