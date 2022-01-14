// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;

public class Limelight extends SubsystemBase {
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = limelightTable.getEntry("tv");
  NetworkTableEntry tx = limelightTable.getEntry("tx");
  NetworkTableEntry ty = limelightTable.getEntry("ty");
  NetworkTableEntry ta = limelightTable.getEntry("ta");

  //AxisCamera limelightVideo = CameraServer.addAxisCamera("limelight", "10.10.24.11:5800");
  //DP: Try this:  (may need to add .getInstanct(), may need to reference a .mjpg file)
  //MjpegServer limelightVideo = CameraServer.startAutomaticCapture(new HttpCamera("limelightCamera",
  //                                                                        "http://10.10.24.11:5800"));

  // or try this (and add tab.add(Limelight.getFeed() in RobotContianer))
  SendableCameraWrapper limelightVideo = SendableCameraWrapper.wrap(new HttpCamera("limelightCamera",
                                                                  "http://10.10.24.11:5800"));   


  double hasTarget = tv.getDouble(0.0);
  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double a = ta.getDouble(0.0);

  /** Creates a new Limelight. */
  public Limelight() {
   // CameraServer.startAutomaticCapture(limelightVideo);

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

  public SendableCameraWrapper getFeed() {
    return limelightVideo;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hasTarget = tv.getDouble(0.0);
  }
}
