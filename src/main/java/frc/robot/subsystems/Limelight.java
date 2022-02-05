// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase {
  private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry tv = limelightTable.getEntry("tv");
  private final NetworkTableEntry tx = limelightTable.getEntry("tx");
  private final NetworkTableEntry ty = limelightTable.getEntry("ty");
  private final NetworkTableEntry ta = limelightTable.getEntry("ta");
  private final NetworkTableEntry pipeline = limelightTable.getEntry("pipeline");
  private final NetworkTableEntry ledMode = limelightTable.getEntry("ledMode");

  private final HttpCamera camera = new HttpCamera(
      "limelight", "http://10.10.24.11:5800", HttpCamera.HttpCameraKind.kMJPGStreamer
  );

  private final PIDController limelightPID = new PIDController(LimelightConstants.kP, LimelightConstants.kI, LimelightConstants.kD);

  /** Creates a new Limelight. */
  public Limelight() {
    disableLeds();
    setDriverPipe();
  }

  public double getXOffset() {
    return tx.getDouble(0.0);
  }

  public double getYOffset() {
    return ty.getDouble(0.0);
  }

  public double getArea() {
    return ta.getDouble(0.0);
  }

  public boolean hasTarget() {
    if(tv.getDouble(0.0) == 1.0) {
      return true;
    } else {
      return false;
    }
  }

  public void enableLeds() {
    ledMode.setDouble(3.0);
  }

  public void disableLeds() {
    ledMode.setDouble(0.0);
  }

  public boolean areLedsOn() {
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

  public void setPipeline(int id) {
    pipeline.setDouble(id);
  }

  public void setDriverPipe() {
    pipeline.setDouble(Constants.LimelightConstants.driverPipe);
  }

  public void setTargetPipe() {
    pipeline.setDouble(Constants.LimelightConstants.targetPipe);
  }

  public HttpCamera getFeed() {
    return camera;
  }

  public PIDController getPIDController(){
    return limelightPID;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
