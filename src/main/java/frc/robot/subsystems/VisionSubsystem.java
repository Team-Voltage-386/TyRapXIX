/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.BallVision;



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
  CvSource blurOutputStream = CameraServer.getInstance().putVideo("Blur", resolutionWidth, resolutionHeight);
  CvSource hSVScaleOutputStream = CameraServer.getInstance().putVideo("Edges", resolutionWidth, resolutionHeight);
  CvSource blackWhiteOutputStream = CameraServer.getInstance().putVideo("BlackWhite", resolutionWidth, resolutionHeight);
  CvSource erodeDilateOutputStream = CameraServer.getInstance().putVideo("ErodeDilate", resolutionWidth, resolutionHeight);
  CvSource contoursOutputStream = CameraServer.getInstance().putVideo("Rectangles", resolutionWidth, resolutionHeight);

  Mat originalImage = new Mat();
  Mat alteredImage = new Mat();
  Mat edgeImage = new Mat();
  Mat grayscaleImage = new Mat();


  public Mat hierarchy = new Mat(), mat = new Mat(), image = new Mat();
  public List<MatOfPoint> finalContours = new ArrayList<>();
  public List<RotatedRect> rects = new ArrayList<>();


  Size blurSize = new Size(9, 9);
  Scalar colorStart = new Scalar(0, 99, 165);
  Scalar colorEnd = new Scalar(20, 253, 255);
  Size erodeSize = new Size(10, 10);
  Size dilateSize = new Size(10, 10);
  Size edgeDilateSize = new Size(4, 4);
  Size edgeErodeSize = new Size(3,3);



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new BallVision());
  }

  public void BallVision(){
    //Gets the unprocessed image
    cvSink.grabFrame(originalImage);
    //Blurs the image for ease of processing 1
    Imgproc.blur(originalImage,alteredImage,blurSize);
    blurOutputStream.putFrame(alteredImage);

    //Converts from RGB scale to HSV scale because HSV is more useful 2
    Imgproc.cvtColor(alteredImage,alteredImage,Imgproc.COLOR_BGR2HSV);
    hSVScaleOutputStream.putFrame(alteredImage);

    //Converts the mat to grayscale (black/white) where pixels in the given range appear white 3
    Core.inRange(alteredImage, colorStart, colorEnd, alteredImage);
    blackWhiteOutputStream.putFrame(alteredImage);

    //Erode and then dilate to sharpen the corners 4
    Imgproc.erode(alteredImage,alteredImage,Imgproc.getStructuringElement(Imgproc.MORPH_RECT, erodeSize));
    Imgproc.dilate(alteredImage,alteredImage,Imgproc.getStructuringElement(Imgproc.MORPH_RECT, dilateSize));
    erodeDilateOutputStream.putFrame(alteredImage);

    //Makes a greyscale version of the original image for edge detection 5
    Imgproc.cvtColor(originalImage, grayscaleImage, Imgproc.COLOR_BGR2GRAY);
    //Dilate and erode to convers separate edges to continuous lines
    Imgproc.dilate(grayscaleImage, grayscaleImage, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, edgeDilateSize));
    Imgproc.erode(grayscaleImage, grayscaleImage, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));

    //Erases the edge pixils from the color filtered image
    Core.bitwise_not(grayscaleImage, grayscaleImage);
    Core.bitwise_and(mat, grayscaleImage, mat);

    finalContours.clear();
    //FInds the outlines of the white rectangles of the current image
    Imgproc.findContours(mat, finalContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
    rects.clear();
    //Makes rectangle objects for each of the contours
    for (int i = 0; i < finalContours.size(); i++) {
      RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(finalContours.get(i).toArray()));
      if (rect.size.height > 10 && rect.size.width > 10 && rect.angle < 10)
        rects.add(rect);

    }
      //Finds the center of the rectangle
      double y = rects.get(0).center.y;
      double x = rects.get(0).center.x;
      SmartDashboard.putNumber("X Value", x);
      SmartDashboard.putNumber("Y Value", y);

      Point[] vertices = new Point[4];
      rects.get(0).points(vertices);
      MatOfPoint points = new MatOfPoint(vertices);
      Imgproc.drawContours(originalImage, Arrays.asList(points), -1, new Scalar(0, 255, 0), 5);
      contoursOutputStream.putFrame(originalImage);

      double distanceFromCenter = x-160;
      double angleFromCenter = Math.atan(distanceFromCenter);




  }

  

}
