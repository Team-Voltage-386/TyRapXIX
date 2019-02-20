package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class ManualClimb extends Command {
  public ManualClimb() {
    requires(Robot.endgameClimbSubsystem);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    // SmartDashboard.putNumber("Left Joystick Value Manip",
    // OI.xboxManipControl.getRawAxis(OI.MANIP_LEFT_JOYSTICK_VERTICAL));
    Robot.endgameClimbSubsystem.setClimbArmSpeeds(OI.xboxTestControl.getRawAxis(OI.MANIP_RIGHT_JOYSTICK_VERTICAL));
    if (OI.xboxTestControl.getRawButton(8)) {
      Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(.6);
    } else if (OI.xboxTestControl.getRawButton(7)) {
      Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(-.6);
    } else {
      Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(0);
    }

    if (OI.xboxTestControl.getRawAxis(OI.MANIP_LEFT_JOYSTICK_VERTICAL) > 0.05
        || OI.xboxTestControl.getRawAxis(OI.MANIP_LEFT_JOYSTICK_VERTICAL) < -0.05) {
      Robot.endgameClimbSubsystem.setElevatorSpeed(OI.xboxTestControl.getRawAxis(OI.MANIP_LEFT_JOYSTICK_VERTICAL));
    } else if (Robot.endgameClimbSubsystem.getElevatorLimitSwitch()) {
      Robot.endgameClimbSubsystem.setElevatorSpeed(0.15);
    } else {
      Robot.endgameClimbSubsystem.setElevatorSpeed(0);
    }
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
