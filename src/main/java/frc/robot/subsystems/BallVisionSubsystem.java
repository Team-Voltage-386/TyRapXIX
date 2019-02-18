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
import java.lang.Math;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
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
import frc.robot.commands.Turn2Ball;

/**
 * Add your docs here.
 */
public class BallVisionSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public BallVisionSubsystem() {
    usbCamera.setExposureManual(1);
    usbCamera.setFPS(120);
  }

  public int resolutionWidth = 320;
  public int resolutionHeight = 240;

  UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture();
  CvSink cvSink = CameraServer.getInstance().getVideo();
  CvSource blurOutputStream = CameraServer.getInstance().putVideo("Blur", resolutionWidth, resolutionHeight);
  CvSource hSVScaleOutputStream = CameraServer.getInstance().putVideo("Edges", resolutionWidth, resolutionHeight);
  CvSource blackWhiteOutputStream = CameraServer.getInstance().putVideo("BlackWhite", resolutionWidth,
      resolutionHeight);
  CvSource erodeDilateOutputStream = CameraServer.getInstance().putVideo("ErodeDilate", resolutionWidth,
      resolutionHeight);
  CvSource contoursOutputStream = CameraServer.getInstance().putVideo("Rectangles", resolutionWidth, resolutionHeight);

  Mat originalImage = new Mat();
  Mat alteredImage = new Mat();
  Mat edgeImage = new Mat();
  Mat grayscaleImage = new Mat();

  public Mat hierarchy = new Mat();
  public Mat mat = new Mat();
  public Mat image = new Mat();
  public List<MatOfPoint> finalContours = new ArrayList<MatOfPoint>();
  public List<Rect> rects = new ArrayList<Rect>();

  Size blurSize = new Size(9, 9);
  Scalar colorStart = new Scalar(0, 150, 20);
  Scalar colorEnd = new Scalar(40, 255, 255);
  Size erodeSize = new Size(13, 13);
  Size dilateSize = new Size(10, 10);
  Size edgeDilateSize = new Size(4, 4);
  Size edgeErodeSize = new Size(3, 3);

  public Double rightCenter = 0.0;
  Rect rightRect = new Rect(0, 0, 0, 0);

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new Turn2Ball());
  }

  public Rect ballVision() {
    // Gets the unprocessed image
    cvSink.grabFrame(originalImage);
    Core.flip(originalImage, originalImage, -1);
    // Blurs the image for ease of processing 1
    Imgproc.blur(originalImage, alteredImage, blurSize);
    blurOutputStream.putFrame(alteredImage);

    // Converts from RGB scale to HSV scale because HSV is more useful 2
    Imgproc.cvtColor(alteredImage, alteredImage, Imgproc.COLOR_BGR2HSV);
    hSVScaleOutputStream.putFrame(alteredImage);

    // Converts the mat to grayscale (black/white) where pixels in the given range
    // appear white 3
    Core.inRange(alteredImage, colorStart, colorEnd, alteredImage);
    blackWhiteOutputStream.putFrame(alteredImage);

    // Erode and then dilate to sharpen the corners 4
    Imgproc.erode(alteredImage, alteredImage, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, erodeSize));
    Imgproc.dilate(alteredImage, alteredImage, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, dilateSize));
    erodeDilateOutputStream.putFrame(alteredImage);

    finalContours.clear();
    // Finds the outlines of the white rectangles of the current image
    Imgproc.findContours(alteredImage, finalContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
    rects.clear();

    // Makes rectangle objects for each of the contours
    for (int i = 0; i < finalContours.size(); i++) {
      Rect rect = Imgproc.boundingRect(new MatOfPoint2f(finalContours.get(i).toArray()));
      if (rect.height > 10 && rect.width > 10) {
        rects.add(rect);
      }
    }

    if (rects.size() < 1) {
      rightRect = new Rect(0, 0, 0, 0);
      rightCenter = 0.0;
    }

    // Double x = Math.abs(rects.get(i).x + rects.get(i).width / 2 - 160.0);
    // if (x <= rightCenter) {
    // rightRect = rects.get(i);
    // }
    // }
    else {
      rightCenter = Math.abs(rects.get(0).y + rects.get(0).height / 2 - 120.0);
      for (int i = 0; i < rects.size(); i++) {
        Double y = Math.abs(rects.get(i).y + rects.get(i).height / 2 - 120.0);
        if (y <= rightCenter) {
          rightRect = rects.get(i);
        }
      }
      rightCenter = Math.abs(rightRect.x + rects.get(0).width / 2 - 160.0);
      rects.remove(rightRect);
    }
    // Imgproc.rectangle(originalImage, new Point(rightRect.x, rightRect.y), new
    // Point(rightRect.x+rightRect.width, rightRect.y+rightRect.height), new
    // Scalar(255, 255, 255));

    Imgproc.rectangle(originalImage, new Point(rightRect.x, rightRect.y),
        new Point(rightRect.x + rightRect.width, rightRect.y + rightRect.height), new Scalar(0, 255, 0));
    for (int i = 0; i < rects.size(); i++) {
      Imgproc.rectangle(originalImage, new Point(rects.get(i).x, rects.get(i).y),
          new Point(rects.get(i).x + rects.get(i).width, rects.get(i).y + rects.get(i).height),
          new Scalar(255, 255, 255));
    }

    contoursOutputStream.putFrame(originalImage);
    // double distanceFromCenter = x-160;
    // double angleFromCenter = Math.atan(distanceFromCenter);
    // double error;
    // if ((rightCenter == 0.0)) {
    // error = 0.0;
    // } else {
    // error = (rightCenter - originalImage.width() / 2 + 0.0);
    // }
    return rightRect;
  }
}
