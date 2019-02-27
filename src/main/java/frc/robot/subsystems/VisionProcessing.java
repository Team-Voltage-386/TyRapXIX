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
// import frc.robot.commands.TurnToTarget;
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
  Size erodeSize = new Size(8, 8);
  Size dilateSize = new Size(5, 5);
  Size edgeDilateSize = new Size(4, 4);

  double[][] matArrayK = new double[][] { { 90.90096432173249, 0.0, 170.0017242958659 },
      { 0.0, 121.0497364596671, 122.79000533774406 }, { 0.0, 0.0, 1.0 } };
  double[][] matArrayD = new double[][] { { -0.0422236020563117 }, { 0.0060033988459270386 }, { -0.019605213853455674 },
      { 0.006526951748079306 } };

  public double currenterror, error1, error2, finalerror;
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
    colorStart = new Scalar(80, 40, 125);
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

      // TestOutputStream.putFrame(base);
      Imgproc.undistort(base, mat, flatMatK, flatMatD);
      Imgproc.undistort(base, flatBase, flatMatK, flatMatD);
      FlatOutputStream.putFrame(mat);

      // Blurs the image for ease of processing
      Imgproc.blur(mat, mat, blurSize);
      // Converts from the RGB scale to HSV because HSV is more useful
      Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);

      // COnverst Mat to a black and white image where pixils in the given range
      // appear white
      Core.inRange(mat, colorStart, colorEnd, mat);

      Imgproc.erode(mat, mat, erodeElement);
      Imgproc.dilate(mat, mat, dilateElement);
      // Imgproc.erode(mat, mat, erodeElement);
      // Imgproc.dilate(mat, mat, dilateElement);

      List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
      hierarchy = new Mat();

      // Find contours
      Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

      ArrayList<Rect> rects = new ArrayList<>();

      for (int i = 0; i < contours.size(); i++) {
        Rect rect = Imgproc.boundingRect(new MatOfPoint2f(contours.get(i).toArray()));
        rects.add(rect);
      }

      Imgproc.drawContours(flatBase, contours, -1, new Scalar(255, 255, 255));

      // for (int i = 0; i < rects.size(); i++) {
      // Point[] vertices = new Point[4];
      // rects.get(i).points(vertices);
      // MatOfPoint points = new MatOfPoint(vertices);
      // Imgproc.drawContours(flatBase, Arrays.asList(points), -1, new Scalar(i * (255
      // / rects.size()), 255, 255), 2);
      // }

      if (rects.size() > 1) {
        error1 = (rects.get(0).x + rects.get(0).width / 2) - resolutionWidth / 2;
        error2 = (rects.get(1).x + rects.get(1).width / 2) - resolutionWidth / 2;
        if (Math.abs(error1) < Math.abs(error2)) {
          currenterror = error1;
          error1 = error2;
          error2 = currenterror;
        }

        for (int i = 2; i < rects.size(); i++) {
          currenterror = (rects.get(i).x + rects.get(i).width / 2) - resolutionWidth / 2;
          if (Math.abs(currenterror) < Math.abs(error1)) {
            if (Math.abs(currenterror) < Math.abs(error2)) {
              error1 = error2;
              error2 = currenterror;
            } else {
              error1 = currenterror;
            }
          }

        }
        finalerror = (error1 + error2) / 2;

      } else {
        finalerror = 1;
      }
      // rects = sortRectX(rects);

      // for (int i = 0; i < rects.size() - 1; i++) {
      // if (getAngle(rects.get(i)) > 0 && getAngle(rects.get(i + 1)) < 0) {
      // pair = new RotatedRect[2];
      // pair[0] = rects.get(i);
      // pair[1] = rects.get(i + 1);
      // pairs.add(pair);
      // i++;
      // }
      // }

      // SmartDashboard.putNumber("Number of Pairs", pairs.size());

      // for (int i = 0; i < pairs.size(); i++) {
      // Imgproc.line(flatBase, pairs.get(i)[0].center, pairs.get(i)[1].center, new
      // Scalar(0, 255, 255));
      // }

      HSVOutputStream.putFrame(flatBase);
      TestOutputStream.putFrame(mat);
    }
    return finalerror;
  }

  // public static double getPairCenter(`Rect[] pair) {
  // return (pair[0].center.x + pair[1].center.x) / 2;
  // }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // setDefaultCommand(new VisionProcess());
  }
}
