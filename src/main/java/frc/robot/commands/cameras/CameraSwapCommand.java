/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cameras;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class CameraSwapCommand extends InstantCommand {

  boolean allowCam1 = false;

  public CameraSwapCommand() {
    super();
    requires(Robot.cameraSubsystem);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    allowCam1 = !allowCam1;
    Robot.cameraSubsystem.cameraSwitch(allowCam1);
  }

}
