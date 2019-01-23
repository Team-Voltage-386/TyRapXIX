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
import org.opencv.core.Rect;
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
    usbCamera.setExposureManual(15);
  }
  public UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture();
  
  //public AxisCamera axisCamera = CameraServer.getInstance().addAxisCamera("10.3.86.23");
  public CvSink cvSink = CameraServer.getInstance().getVideo();
  public CvSource HSVOutputStream = CameraServer.getInstance().putVideo("Edges", resolutionWidth, resolutionHeight);
  public CvSource TestOutputStream = CameraServer.getInstance().putVideo("Final", resolutionWidth, resolutionHeight);

	Mat base = new Mat();
	Mat mat = new Mat();
	Mat grey = new Mat();
  Mat edges = new Mat();
  Mat hierarchy;

	Size blurSize = new Size(9, 9);
	Scalar colorStart = new Scalar(0, 0, 100);
	Scalar colorEnd = new Scalar(255, 255, 255);
	Size erodeSize = new Size(10, 10);
	Size dilateSize = new Size(10, 10);
  Size edgeDilateSize = new Size(4, 4);

  public double getAngle(RotatedRect calculatedRect){
    if(calculatedRect.size.width < calculatedRect.size.height){
      return calculatedRect.angle+90;
    }else{
      return calculatedRect.angle;
    }
  }

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
        Imgproc.drawContours(base, Arrays.asList(points), -1, new Scalar(140,255,255), 2);
      }

      ArrayList<RotatedRect> bachelors = new ArrayList<RotatedRect>(); 
      ArrayList<RotatedRect> bachelorettes = new ArrayList<RotatedRect>(); 
      double bestDistance;
      RotatedRect bachelor;
      RotatedRect bachelorette;
      boolean paired;
      

      // for(int n = 0 ; n < rects.size() ; n++){
      //   for(int i = 0 ; i < rects.size() ; i++){
      //     distance = Math.abs(rects.get(n).center.x - rects.get(i).center.x);
      //     if(getAngle(rects.get(n)) < 0 && getAngle(rects.get(i)) > 0){
      //       possiblePairs.add(rects.get(i));
      //       possibleDistance.add(distance);
      //     }
      //   }
      //   if(possiblePairs.size() > 1){
      //     bestRect = possiblePairs.get(0);
      //     bestDistance = possibleDistance.get(0);
      //     for(int x = 0 ; x < possiblePairs.size() ; x++){
      //       if(possibleDistance.get(x) < bestDistance){
      //         bestRect = possiblePairs.get(x);
      //         bestDistance = possibleDistance.get(x);
      //       }
      //     }
      //     pair[0] = rects.get(n);
      //     pair[1] = bestRect;
      //   }
      //   else if(possiblePairs.size() == 1){
      //     pair[0] = rects.get(n);
      //     pair[1] = possiblePairs.get(0);
      //   }
      // }

      while(rects.size() > 0){
        bestDistance = mat.width();
        bachelor = rects.get(0);
        bachelorette = rects.get(0);
        paired = false;

        for(int i = 1; i < rects.size() ; i++){
          if( (getAngle(bachelor)*getAngle(rects.get(i)) < 0) 
          && Math.abs(rects.get(i).center.x - bachelor.center.x) < bestDistance){

            if( (getAngle(rects.get(i)) < 0 && rects.get(i).center.x < bachelor.center.x) ||
            (getAngle(rects.get(i)) > 0 && rects.get(i).center.x > bachelor.center.x) ){

              bachelorette = rects.get(i);
              bestDistance = Math.abs(rects.get(i).center.x - bachelor.center.x);
              paired = true;
            }
          }
        }

        if(paired){
          SmartDashboard.putNumber("Possible Pair Rect 0 x value", bachelor.center.x);
          SmartDashboard.putNumber("Possible Pair Rect 1 x value", bachelorette.center.x);
          bachelors.add(bachelor);
          bachelorettes.add(bachelorette);

          SmartDashboard.putNumber("Bachelor Index", rects.indexOf(bachelor));
          SmartDashboard.putNumber("Bachelorette Index",rects.indexOf(bachelorette));
          rects.remove(bachelor);
          rects.remove(bachelorette);
          
        }else{
          rects.remove(bachelor);
        }
      }

      SmartDashboard.putNumber("Number of pairs",bachelors.size());
      SmartDashboard.putNumber("Number of rects",rects.size());

      for(int i = 0 ; i < bachelors.size() ; i++){
        Imgproc.line(base, bachelors.get(i).center, bachelorettes.get(i).center, new Scalar(0,255,255));
        SmartDashboard.putNumber("Pair " + i + ", Rect 0 x value", bachelors.get(i).center.x);
        SmartDashboard.putNumber("Pair " + i + ", Rect 1 x value", bachelorettes.get(i).center.x);
      }
          
      HSVOutputStream.putFrame(base);

    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new VisionProcess());
  }
}
