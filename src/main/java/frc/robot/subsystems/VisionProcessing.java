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
    usbCamera.setExposureManual(15);
  }
  public UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture();
  
  //public AxisCamera axisCamera = CameraServer.getInstance().addAxisCamera("10.3.86.23");
  public CvSink cvSink = CameraServer.getInstance().getVideo();
  public CvSource HSVOutputStream = CameraServer.getInstance().putVideo("Final", resolutionWidth, resolutionHeight);
  public CvSource TestOutputStream = CameraServer.getInstance().putVideo("Edges", resolutionWidth, resolutionHeight);

	public Mat base = new Mat();
	public Mat mat = new Mat();
	Mat grey = new Mat();
  Mat edges = new Mat();
  Mat hierarchy;

	Size blurSize = new Size(9, 9);
	Scalar colorStart = new Scalar(75, 100, 100);
	Scalar colorEnd = new Scalar(100, 255, 255);
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

  public ArrayList<RotatedRect> sortRectX(ArrayList<RotatedRect> rects){
    int n = rects.size(), min;
    RotatedRect temp;

    for(int i = 0 ; i<n-1 ; i++){
			min = i;

			for(int j = i+1 ; j<n ; j++){
				if(rects.get(j).center.x < rects.get(min).center.x){
					min = j;
				}
			}

			if(min!=i){
				temp = rects.get(min);
				rects.set(min, rects.get(i));
				rects.set(i, temp);
			}
				
		}

    return rects;
  }
  
  public ArrayList<RotatedRect[]> visionProcess(){

    //Recive the inital image
    cvSink.grabFrame(base);

    //Set up ArrayList for pairs
    ArrayList<RotatedRect[]> pairs = new ArrayList<RotatedRect[]>(); 
    RotatedRect[] pair = new RotatedRect[2];

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

      ArrayList<RotatedRect> rects = new ArrayList<>();

      for(int i = 0 ; i < contours.size() ; i++){
        RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));
        rects.add(rect);
      }

      rects = sortRectX(rects);

      for(int i = 0 ; i<rects.size()-1 ; i++){
        if(getAngle(rects.get(i))<0 && getAngle(rects.get(i+1))>0){
          pair[0] = rects.get(i);
          pair[1] = rects.get(i+1);
          pairs.add(pair);
          i++;
        }
      }

      for(int i = 0 ; i<pairs.size() ; i++){
        Imgproc.line(base, pairs.get(i)[0].center, pairs.get(i)[1].center, new Scalar(0,255,255));
      }

      for(int i = 0 ; i<rects.size() ; i++){
        Point[] vertices = new Point[4];
        rects.get(i).points(vertices);
        MatOfPoint points = new MatOfPoint(vertices);
        Imgproc.drawContours(base, Arrays.asList(points), -1, new Scalar(i*(255/rects.size()),255,255), 2);
      }
          
      HSVOutputStream.putFrame(base);
      TestOutputStream.putFrame(mat);
    }
    return pairs;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new VisionProcess());
  }
}
