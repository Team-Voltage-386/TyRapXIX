/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.VisionProcess;

/**
 * Add your docs here.
 */
public class VisionProcessing extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public int resolutionWidth = 320;
  public int resolutionHeight = 240;

  public VisionProcessing(){
    usbCamera.setResolution(resolutionWidth, resolutionHeight);
    usbCamera.setFPS(30);
  }
  public UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture();
  
  //public AxisCamera axisCamera = CameraServer.getInstance().addAxisCamera("10.3.86.23");
  public CvSink cvSink = CameraServer.getInstance().getVideo();
  public CvSource HSVOutputStream = CameraServer.getInstance().putVideo("Edges", resolutionWidth, resolutionHeight);

  Mat mat = new Mat();
  Mat hierarchy = new Mat();
  Mat base = new Mat();

  Size blurSize = new Size(9, 9);
  Scalar colorStart = new Scalar(20, 100, 75);
  Scalar colorEnd = new Scalar(30, 255, 255);
  
  Rect[] rects;

  public void visionProcess(){

    //Recive the inital image
    cvSink.grabFrame(base);
    //Blurs the image for ease of processing
    Imgproc.blur(base, mat, blurSize);
    //Converts from the RGB scale to HSV because HSV is more useful
    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
    //COnverst Mat to a black and white image where pixils in the given range appear white
    Core.inRange(mat, colorStart, colorEnd, mat);

    HSVOutputStream.putFrame(mat);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new VisionProcess());
  }
}
