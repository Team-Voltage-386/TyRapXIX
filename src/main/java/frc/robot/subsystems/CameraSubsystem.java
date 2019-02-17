/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.TurnToBall;

/**
 * Add your docs here.
 */
public class CameraSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public int resolutionWidth = 640;
  public int resolutionHeight = 240;
  UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture("Fisheye",0);
  CvSink cvSink = CameraServer.getInstance().getVideo();
  CvSource testOutputStream = CameraServer.getInstance().putVideo("Test",resolutionHeight,resolutionWidth);

  Mat originalImage = new Mat();

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new TurnToBall());
  }

  public void testVision(){
    cvSink.grabFrame(originalImage);
    testOutputStream.putFrame(originalImage);
  }

  


}
