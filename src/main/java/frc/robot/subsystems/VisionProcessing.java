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

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
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
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.VisionProcess;

/**
 * Add your docs here.
 */
public class VisionProcessing extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public int resolutionWidth = 320;
  public int resolutionHeight = 240;
  public UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture();

  public VisionProcessing() {
    usbCamera.setResolution(resolutionWidth, resolutionHeight);
    usbCamera.setFPS(30);
    usbCamera.setExposureAuto();
  }


  // public AxisCamera axisCamera =
  // CameraServer.getInstance().addAxisCamera("10.3.86.23"); 
  public CvSink cvSink = CameraServer.getInstance().getVideo();
  public CvSource HSVOutputStream = CameraServer.getInstance().putVideo("Final", resolutionWidth, resolutionHeight);
  public CvSource TestOutputStream = CameraServer.getInstance().putVideo("Edges", resolutionWidth, resolutionHeight);
  public CvSource ErodeDilateStream = CameraServer.getInstance().putVideo("ErodeDilate", resolutionWidth,
      resolutionHeight);
  public CvSource ContrastOutputStream = CameraServer.getInstance().putVideo("Contrast", resolutionWidth,
      resolutionHeight);
  public CvSource contoursOutputStream = CameraServer.getInstance().putVideo("Rectangles", resolutionWidth, resolutionHeight);

  public Mat base = new Mat();
  Mat contrast = new Mat();
  Mat grey = new Mat();
  Mat edges = new Mat();
  public Mat erode = new Mat();


  Size blurSize = new Size(9, 9);
  Scalar colorStart;
  Scalar colorEnd;

  Size edgeDilateSize = new Size(4, 4);
  public Mat hierarchy = new Mat();
  public Mat mat = new Mat();
  public Mat image = new Mat();
  public List<MatOfPoint> finalContours = new ArrayList<MatOfPoint>();
  public List<Rect> rects = new ArrayList<Rect>();
  public Double rightCenter = 0.0;
  Rect rightRect = new Rect(0, 0, 0, 0);

  Size dilateSize = new Size(SmartDashboard.getNumber("DilateSize1", 30), SmartDashboard.getNumber("DilateSize2", 30));
  Size erodeSize = new Size(SmartDashboard.getNumber("ErodeSize1", 80), SmartDashboard.getNumber("ErodeSize2", 80));
  Size defaultDilate = new Size(SmartDashboard.getNumber("DefaultDilate", 10),
      SmartDashboard.getNumber("DefaultDilate", 10));

  // public double getAngle(RotatedRect calculatedRect) {
  // if (calculatedRect.size.width < calculatedRect.size.height) {
  // return calculatedRect.angle + 90;
  // } else {
  // return calculatedRect.angle;
  // }
  // }

  // public ArrayList<RotatedRect> sortRectX(ArrayList<RotatedRect> rects) {
  // int n = rects.size(), min;
  // RotatedRect temp;

  // for (int i = 0; i < n - 1; i++) {
  // min = i;

  // for (int j = i + 1; j < n; j++) {
  // if (rects.get(j).center.x < rects.get(min).center.x) {
  // min = j;
  // }
  // }

  // if (min != i) {
  // temp = rects.get(min);
  // rects.set(min, rects.get(i));
  // rects.set(i, temp);
  // }

  // }

  // return rects;
  // }

  public double visionProcess() {

    // Vision Thresholds
    colorStart = new Scalar(SmartDashboard.getNumber("Start H", 125), SmartDashboard.getNumber("Start S", 0),
        SmartDashboard.getNumber("Start V", 0));
    colorEnd = new Scalar(SmartDashboard.getNumber("End H", 200), SmartDashboard.getNumber("End S", 255),
        SmartDashboard.getNumber("End V", 255));
    double alpha = SmartDashboard.getNumber("alpha", 2.0);
    double beta = SmartDashboard.getNumber("beta", 40);

    // Recive the inital image
    cvSink.grabFrame(base);



    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, dilateSize);
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, erodeSize);

    if (!base.empty()) {
      // Blurs the image for ease of processing
      Imgproc.blur(base, mat, blurSize);
      // Converts from the RGB scale to HSV because HSV is more useful
      Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);

      // Hatch Vision stuff

      base.convertTo(contrast, -1, alpha, beta);
      base.convertTo(mat, -1, alpha, beta);

      // COnverst Mat to a black and white image where pixils in the given range
      // appear white
      Core.inRange(mat, colorStart, colorEnd, mat);
      Imgproc.dilate(mat, erode, dilateElement);
      Imgproc.erode(erode, erode, erodeElement);
      finalContours.clear();
      // Finds the outlines of the white rectangles of the current image
      Imgproc.findContours(erode,finalContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
      rects.clear();
  
      // Makes rectangle objects for each of the contours
      for (int i = 0; i < finalContours.size(); i++) {
        Rect rect = Imgproc.boundingRect(new MatOfPoint2f(finalContours.get(i).toArray()));
        if (rect.height > 10 && rect.width > 10) {
          rects.add(rect);
        }
      }
  
      if (rects.size() < 1) {
        rightCenter = 0.0;
  
      }
      // else {
      // rightCenter = Math.abs(rects.get(0).x + rects.get(0).width / 2 - 160.0);
      // for (int i = 0; i < rects.size(); i++) {
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
  
      Imgproc.rectangle(base, new Point(rightRect.x, rightRect.y),
          new Point(rightRect.x + rightRect.width, rightRect.y + rightRect.height), new Scalar(0, 255, 0));
      for (int i = 0; i < rects.size(); i++) {
        Imgproc.rectangle(base, new Point(rects.get(i).x, rects.get(i).y),
            new Point(rects.get(i).x + rects.get(i).width, rects.get(i).y + rects.get(i).height),
            new Scalar(255, 255, 255));
      }
  
      contoursOutputStream.putFrame(base);
      // double distanceFromCenter = x-160;
      // double angleFromCenter = Math.atan(distanceFromCenter);
      double error;
      if ((rightCenter == 0.0)) {
        error = 0.0;
      } else {
        // Not sure if error = rightCenter
        error = (rightCenter - base.width() / 2.0);
      }
  
      HSVOutputStream.putFrame(base);
      TestOutputStream.putFrame(mat);
      ErodeDilateStream.putFrame(erode);
      ContrastOutputStream.putFrame(contrast);

      return error;


      // List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
      // hierarchy = new Mat();

      // // Find contours
      // Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_CCOMP,
      // Imgproc.CHAIN_APPROX_SIMPLE);

      // ArrayList<RotatedRect> rects = new ArrayList<>();

      // for (int i = 0; i < contours.size(); i++) {
      // RotatedRect rect = Imgproc.minAreaRect(new
      // MatOfPoint2f(contours.get(i).toArray()));
      // rects.add(rect);
      // }

      // for (int i = 0; i < rects.size(); i++) {
      // Point[] vertices = new Point[4];
      // rects.get(i).points(vertices);
      // MatOfPoint points = new MatOfPoint(vertices);
      // Imgproc.drawContours(base, Arrays.asList(points), -1, new Scalar(i * (255 /
      // rects.size()), 255, 255), 2);
      // }

      // rects = sortRectX(rects);

      // for (int i = 0; i < rects.size() - 1; i++) {
      // if (getAngle(rects.get(i)) < 0 && getAngle(rects.get(i + 1)) > 0) {
      // pair = new RotatedRect[2];
      // pair[0] = rects.get(i);
      // pair[1] = rects.get(i + 1);
      // pairs.add(pair);
      // i++;
      // }
      // }

      // SmartDashboard.putNumber("Number of Pairs", pairs.size());

      // for (int i = 0; i < pairs.size(); i++) {
      // Imgproc.line(base, pairs.get(i)[0].center, pairs.get(i)[1].center, new
      // Scalar(0, 255, 255));
      // }


    }
    else{
      return 0.0;
    }
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new VisionProcess());
  }
}
