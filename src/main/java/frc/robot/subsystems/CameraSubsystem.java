// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class CameraSubsystem extends SubsystemBase {
  Thread m_visionThread;
  /** Creates a new VideoOverlaySubsystem. */
  public CameraSubsystem() {
    m_visionThread = 
    new Thread(
      () -> {
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(640, 480);
        CvSink cvSink = CameraServer.getVideo();
        CvSource outputStream = CameraServer.putVideo("Line", 640, 480);
        Mat mat = new Mat();
        while (!Thread.interrupted()) {
          if (cvSink.grabFrame(mat) == 0) {
            outputStream.notifyError(cvSink.getError());
            continue;
          }
          Imgproc.line(mat, new Point((mat.size().width / 2), mat.size().height), new Point((mat.size().width / 2), 0), new Scalar(0, 0, 0), 2); // TODO: Implement sensor distance into width
          outputStream.putFrame(mat);
        }
      });
      m_visionThread.setDaemon(true);
      m_visionThread.start();
  }
}
