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
import org.opencv.core.CvType;
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

  public VisionProcessing() {
    usbCamera.setResolution(resolutionWidth, resolutionHeight);
    usbCamera.setFPS(30);
    usbCamera.setExposureManual(1);
  }

  public UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture();

  // public AxisCamera axisCamera =
  // CameraServer.getInstance().addAxisCamera("10.3.86.23");
  public CvSink cvSink = CameraServer.getInstance().getVideo();
  public CvSource FlatOutputStream = CameraServer.getInstance().putVideo("After", resolutionWidth, resolutionHeight);
  public CvSource HSVOutputStream = CameraServer.getInstance().putVideo("Final", resolutionWidth, resolutionHeight);
  public CvSource TestOutputStream = CameraServer.getInstance().putVideo("Before", resolutionWidth, resolutionHeight);

  public Mat base = new Mat();
  public Mat mat = new Mat();
  public Mat flatBase = new Mat();
  Mat grey = new Mat();
  Mat edges = new Mat();
  Mat hierarchy;
  Mat flatMatK = new Mat(3, 3, CvType.CV_64FC1);
  Mat flatMatD = new Mat(4, 1, CvType.CV_64FC1);
  Mat testMat = new Mat();

  Size blurSize = new Size(9, 9);
  Scalar colorStart;
  Scalar colorEnd;
  Size erodeSize = new Size(10, 10);
  Size dilateSize = new Size(10, 10);
  Size edgeDilateSize = new Size(4, 4);

  double[][] matArrayK = new double[][] { { 90.91014289083918, 0.0, 169.08644162302508 },
      { 0.0, 121.2050237039938, 122.413423825152 }, { 0.0, 0.0, 1.0 } };
  double[][] matArrayD = new double[][] { { -0.03667242122705496 }, { -0.007197203576932934 },
      { -0.010108381049152206 }, { 0.0040492602483107815 } };

  public double getAngle(RotatedRect calculatedRect) {
    if (calculatedRect.size.width < calculatedRect.size.height) {
      return calculatedRect.angle + 90;
    } else {
      return calculatedRect.angle;
    }
  }

  public ArrayList<Rect> sortRectX(ArrayList<Rect> rects) {
    int n = rects.size(), min;
    Rect temp;

    for (int i = 0; i < n - 1; i++) {
      min = i;

      for (int j = i + 1; j < n; j++) {
        if (rects.get(j).x < rects.get(min).x) {
          min = j;
        }
      }

      if (min != i) {
        temp = rects.get(min);
        rects.set(min, rects.get(i));
        rects.set(i, temp);
      }

    }

    return rects;
  }

  public ArrayList<Rect[]> visionProcess() {

    // Vision Thresholds
    colorStart = new Scalar(75, 40, 125);
    colorEnd = new Scalar(140, 255, 255);

    // Recive the inital image
    cvSink.grabFrame(base);

    // Set up ArrayList for pairs
    ArrayList<Rect[]> pairs = new ArrayList<Rect[]>();
    Rect[] pair = new Rect[2];

    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));

    if (!base.empty()) {

      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          flatMatK.put(i, j, new double[] { matArrayK[i][j] });
        }
      }

      for (int m = 0; m < flatMatD.height(); m++) {
        for (int n = 0; n < flatMatD.width(); n++) {
          flatMatD.put(m, n, new double[] { matArrayD[m][n] });
        }
      }

      TestOutputStream.putFrame(base);

      // Blurs the image for ease of processing
      Imgproc.blur(base, mat, blurSize);
      // Converts from the RGB scale to HSV because HSV is more useful
      Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);

      // Converts Mat to a black and white image where pixils in the given range
      // appear white
      Core.inRange(mat, colorStart, colorEnd, mat);

      Imgproc.undistort(mat, testMat, flatMatK, flatMatD);
      Imgproc.undistort(mat, flatBase, flatMatK, flatMatD);
      FlatOutputStream.putFrame(testMat);

      // Imgproc.erode(mat, mat, dilateElement);
      // Imgproc.dilate(mat, mat, erodeElement);

      List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
      hierarchy = new Mat();

      // Find contours
      Imgproc.findContours(testMat, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

      ArrayList<Rect> rects = new ArrayList<>();

      for (int i = 0; i < contours.size(); i++) {
        Rect rect = Imgproc.boundingRect(new MatOfPoint2f(contours.get(i).toArray()));
        rects.add(rect);
      }

      for (int i = 0; i < rects.size(); i++) {
        /*Point[] vertices = new Point[4];
        rects.get(i).points(vertices);
        MatOfPoint points = new MatOfPoint(vertices);
        Imgproc.drawContours(flatBase, Arrays.asList(points), -1, new Scalar(i * (255 / rects.size()), 255, 255), 2);*/
        Imgproc.rectangle(flatBase, rects.get(i).br(), rects.get(i).tl(), new Scalar(i * (255 / rects.size()), 255, 255), 2);
      }

      rects = sortRectX(rects);

      for (int i = 0; i < rects.size() - 1; i++) {
          pair = new Rect[2];
          pair[0] = rects.get(i);
          pair[1] = rects.get(i + 1);
          pairs.add(pair);
          i++;
      }

      SmartDashboard.putNumber("Number of Pairs", pairs.size());

      for (int i = 0; i < pairs.size(); i++) {
        Imgproc.line(flatBase, pairs.get(i)[0].br(), pairs.get(i)[1].br(), new Scalar(0, 255, 255));
      }

      HSVOutputStream.putFrame(flatBase);
      TestOutputStream.putFrame(testMat);
    }
    return pairs;
  }

  public static double getPairCenter(Rect[] pair) {
    return (pair[0].x + pair[1].x) / 2;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new VisionProcess());
  }
}
