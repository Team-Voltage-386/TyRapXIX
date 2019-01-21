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
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    usbCamera.setExposureManual(10);
  }
  public UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture();
  
  //public AxisCamera axisCamera = CameraServer.getInstance().addAxisCamera("10.3.86.23");
  public CvSink cvSink = CameraServer.getInstance().getVideo();
  public CvSource HSVOutputStream = CameraServer.getInstance().putVideo("Edges", resolutionWidth, resolutionHeight);

	Mat base = new Mat();
	Mat mat = new Mat();
	Mat grey = new Mat();
  Mat edges = new Mat();
  Mat hierarchy;

	Size blurSize = new Size(9, 9);
	Scalar colorStart = new Scalar(0, 100, 100);
	Scalar colorEnd = new Scalar(255, 255, 255);
	Size erodeSize = new Size(10, 10);
	Size dilateSize = new Size(10, 10);
  Size edgeDilateSize = new Size(4, 4);

  public void visionProcess(){

    //Recive the inital image
    cvSink.grabFrame(base);

    if(!base.empty()){
      //Blurs the image for ease of processing
      Imgproc.blur(base, mat, blurSize);
      //Converts from the RGB scale to HSV because HSV is more useful
      Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
      //COnverst Mat to a black and white image where pixils in the given range appear white
      Core.inRange(mat, colorStart, colorEnd, mat);

      Imgproc.erode(mat, mat, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, erodeSize));
      Imgproc.dilate(mat, mat, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, dilateSize));
    
      List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
      hierarchy = new Mat();

      //Find contours
      Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

      List<RotatedRect> rects = new ArrayList<>();

      for(int i = 0 ; i < contours.size() ; i++){
        RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));
        rects.add(rect);
      }

      for(RotatedRect r : rects){
        Point[] vertices = new Point[4];
        r.points(vertices);
        MatOfPoint points = new MatOfPoint(vertices);
        Imgproc.drawContours(mat, Arrays.asList(points), -1, new Scalar(63,50,156), 5);
      }

      for(int n = 0 ; n < rects.size() ; n++){
        SmartDashboard.putString("Angle for Rect Number", n + " = " + rects.get(n).angle);
      }

      HSVOutputStream.putFrame(mat);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new VisionProcess());
  }
}
