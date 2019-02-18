package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    Robot.endgameClimbSubsystem.setElevatorSpeed(OI.xboxManipControl.getRawAxis(OI.MANIP_LEFT_JOYSTICK_VERTICAL));
    //SmartDashboard.putNumber("Left Joystick Value Manip",
        OI.xboxManipControl.getRawAxis(OI.MANIP_LEFT_JOYSTICK_VERTICAL));
    Robot.endgameClimbSubsystem.setClimbArmSpeeds(OI.xboxManipControl.getRawAxis(OI.MANIP_RIGHT_JOYSTICK_VERTICAL));
    if (OI.xboxManipControl.getRawButton(8)) {
      Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(1);
    } else if (OI.xboxManipControl.getRawButton(7)) {
      Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(-1);
    } else {
      Robot.endgameClimbSubsystem.setElevatorWheelsSpeed(0);
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
