// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Console;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;

public class Limelight extends SubsystemBase {
  private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry tv = limelightTable.getEntry("tv");
  private final NetworkTableEntry tx = limelightTable.getEntry("tx");
  private final NetworkTableEntry ty = limelightTable.getEntry("ty");
  private final NetworkTableEntry ta = limelightTable.getEntry("ta");
  private final NetworkTableEntry ledMode = limelightTable.getEntry("ledMode");

  //AxisCamera limelightVideo = CameraServer.addAxisCamera("limelight", "10.10.24.11:5800");
  //DP: Try this:  (may need to add .getInstanct(), may need to reference a .mjpg file)
  //QT: Does not work, shuffleboard will not accept MjpegServers... using SendableCameraWrapper instead
  //MjpegServer limelightVideo = CameraServer.startAutomaticCapture(new HttpCamera("limelightCamera",
  //                                                                        "http://10.10.24.11:5800"));

  //HttpCamera limelightVideo = new HttpCamera("LimelightCam", "http://10.10.24.11:5800/stream.mjpg");
  // or try this (and add tab.add(Limelight.getFeed() in RobotContianer))
  /*SendableCameraWrapper limelightVideo = SendableCameraWrapper.wrap(
    new HttpCamera("limelightCamera", "http://10.10.24.11:5800/")
  );*/

  private MjpegServer limelightVideo;
  private final HttpCamera camera = new HttpCamera(
      "limelightCamera", "http://10.10.24.11:5800", HttpCamera.HttpCameraKind.kMJPGStreamer
  );

  private double hasTarget = tv.getDouble(0.0);
  private double x = tx.getDouble(0.0);
  private double y = ty.getDouble(0.0);
  private double a = ta.getDouble(0.0);

  /** Creates a new Limelight. */
  public Limelight() {
    disableLeds();

    //camera = CameraServer.startAutomaticCapture(); // Working implementation for UsbCamera (but not working for limelight)
    limelightVideo = CameraServer.startAutomaticCapture(camera);

    System.out.println("X:" + x);
    System.out.println("Y:" + y);
    System.out.println("A:" + a);
  }

  double getX() {
    return tx.getDouble(0.0);
  }

  double getY() {
    return ty.getDouble(0.0);
  }

  double getArea() {
    return ta.getDouble(0.0);
  }

  void enableLeds() {
    ledMode.setDouble(3.0);
  }

  void disableLeds() {
    ledMode.setDouble(0.0);
  }

  boolean areLedsOn() {
    if(ledMode.getDouble(0.0) > 1.0) {
      return true;
    } else {
      return false;
    }
  }

  public void toggleLeds() {
    if(areLedsOn()) {
      disableLeds();
    } else {
      enableLeds();
    }
  }

  // Feeding a UsbCamera seems to work if it is via the CameraServer.startAutomaticCapture()
  public VideoSource getFeed() {
    return limelightVideo.getSource();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hasTarget = tv.getDouble(0.0);
  }
}
