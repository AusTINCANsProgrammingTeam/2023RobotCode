// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class CameraSubsystem extends SubsystemBase {
  private Thread m_visionThread;
  private DataLog errorLog = DataLogManager.getLog();
  private StringLogEntry errors = new StringLogEntry(errorLog, "/errors");
  /** Creates a new VideoOverlaySubsystem. */
  public CameraSubsystem() {
    // Retrieves each individual frame from the robot camera, adds a vertical black line through the middle (might be updated if LIDAR is added to intake), 
    // then puts frame into the output stream to be retrieved in Shuffleboard
    // Taken from the Advanced Camera Server Program on WPILIB
    // https://docs.wpilib.org/en/stable/docs/software/vision-processing/roborio/using-the-cameraserver-on-the-roborio.html#advanced-camera-server-program
    m_visionThread = 
    new Thread(
      () -> {
        UsbCamera camera = CameraServer.startAutomaticCapture(); // Get the UsbCamera from CameraServer
        camera.setResolution(640, 480);  // Set the resolution
        CvSink cvSink = CameraServer.getVideo(); // Get a CvSink. This will capture Mats from the camera
        CvSource outputStream = CameraServer.putVideo("Line", 640, 480); // Setup a CvSource. This will send images back to Shuffleboard.
        Mat mat = new Mat(); // Mat is an array of pixels of an image
        while (!Thread.interrupted()) { // This cannot be 'true'. The program will never exit if it is. This lets the robot stop this thread when restarting robot code or deploying.
        // Tell the CvSink to grab a frame from the camera and put it in the source mat.  If there is an error notify the output.
          if (cvSink.grabFrame(mat) == 0) {
            // Send the output the error.
            outputStream.notifyError(cvSink.getError());
            errors.append("No image received from CvSink");
            continue;
          }
          // Put a line on the image
          Imgproc.line(mat, new Point((mat.size().width / 2), mat.size().height), new Point((mat.size().width / 2), 0), new Scalar(0, 0, 0), 2); // TODO: Implement sensor distance into width
          outputStream.putFrame(mat);  // Give the output stream a new image to display
        }
      });
      m_visionThread.setDaemon(true);
      m_visionThread.start(); 
  }
}