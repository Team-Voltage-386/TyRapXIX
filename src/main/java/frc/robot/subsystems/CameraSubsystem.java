package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.cameras.ContinuousStream;

/**
 * Add your docs here.
 */
public class CameraSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // cameras
  UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);
  UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
  // setting up stream "Switcher"
  CvSink cvSink1 = CameraServer.getInstance().getVideo(camera1);
  CvSink cvSink2 = CameraServer.getInstance().getVideo(camera2);
  CvSource outputStream = CameraServer.getInstance().putVideo("Switcher", 320, 240);

  Mat image = new Mat();

  double cvSinkEnabled = -1;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ContinuousStream());
  }

  public void cameraSwitch(boolean allowCamOne) {
    if (allowCamOne) {
      cvSink2.setEnabled(false);
      cvSink1.setEnabled(true);
      cvSinkEnabled = 1;
    } else {
      cvSink1.setEnabled(false);
      cvSink2.setEnabled(true);
      cvSinkEnabled = 2;
    }
  }

  public void continuousStream() {
    if (cvSinkEnabled == 1) {
      cvSink1.grabFrame(image);
      outputStream.putFrame(image);
    } else if (cvSinkEnabled == 2) {
      cvSink2.grabFrame(image);
      outputStream.putFrame(image);
    }
  }
}
