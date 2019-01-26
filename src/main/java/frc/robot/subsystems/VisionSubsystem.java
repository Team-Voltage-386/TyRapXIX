/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;



/**
 * Add your docs here.
 */
public class VisionSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public int resolutionWidth = 320;
	public int resolutionHeight = 240;

  UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture();
  CvSink cvSink = CameraServer.getInstance().getVideo();
  CvSource HSVOutputStream = CameraServer.getInstance().putVideo("Edges", resolutionWidth, resolutionHeight);
  Mat originalImage,alteredImage,edgeImage,grayscaleImage;

  Size blurSize = new Size(9, 9);
  Scalar colorStart = new Scalar(20, 108, 139);
  Scalar colorEnd = new Scalar(35, 255, 255);
  Size erodeSize = new Size(10, 10);
  Size dilateSize = new Size(10, 10);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void HatchVision(){
    //Gets the unprocessed image
    cvSink.grabFrame(originalImage);
    //Blurs the image for ease of processing
    Imgproc.blur(originalImage,alteredImage,blurSize);
    //Converts from RGB scale to HSV scale because HSV is more useful
    Imgproc.cvtColor(alteredImage,alteredImage,Imgproc.COLOR_BGR2HSV);
    //Converts the mat to grayscale (black/white) where pixels in the given range appear white
    Core.inRange(alteredImage, colorStart, colorEnd, alteredImage);
    //Erode and then dilate to sharpen the corners
    Imgproc.erode(alteredImage,alteredImage,Imgproc.getStructuringElement(Imgproc.MORPH_RECT, erodeSize));
    Imgproc.dilate(alteredImage,alteredImage,Imgproc.getStructuringElement(Imgproc.MORPH_RECT, dilateSize));

    HSVOutputStream.putFrame(alteredImage);


  }


}
