// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry tv = limelightTable.getEntry("tv");
  private final NetworkTableEntry tx = limelightTable.getEntry("tx");
  private final NetworkTableEntry ty = limelightTable.getEntry("ty");
  private final NetworkTableEntry ta = limelightTable.getEntry("ta");
  private final NetworkTableEntry ledMode = limelightTable.getEntry("ledMode");

  private final HttpCamera camera = new HttpCamera(
      "limelight", "http://10.10.24.11:5800/stream.mjpg", HttpCamera.HttpCameraKind.kMJPGStreamer
  );

  private double hasTarget = tv.getDouble(0.0);
  private double x = tx.getDouble(0.0);
  private double y = ty.getDouble(0.0);
  private double a = ta.getDouble(0.0);

  /** Creates a new Limelight. */
  public Limelight() {
    disableLeds();
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

  public void disableLeds() {
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

  public HttpCamera getFeed() {
    return camera;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hasTarget = tv.getDouble(0.0);
  }
}
