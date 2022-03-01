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
    setDriverPipe();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

  /**
   * Turn the Limelight LEDs on.
   */
  public void enableLeds() {
    ledMode.setDouble(3.0);
  }

  /**
   * Turn the Limelight LEDs off.
   */
  public void disableLeds() {
    ledMode.setDouble(0.0);
  }

  /**
   * Determines the state of the Limelight LEDs.
   * @return true if the lights are on, false if they are off
   */
  public boolean areLedsOn() {
    if(ledMode.getDouble(0.0) > 1.0) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Swap the state of the Limelight LEDs
   */
  public void toggleLeds() {
    if(areLedsOn()) {
      disableLeds();
    } else {
      enableLeds();
    }
  }

  /**
   * Sets the Limelight pipeline to that of the specified id
   * @param id the id of the desired pipeline
   * Currently, there are 2 ids that are configured: the driver pipeline (1) and the targeting pipeline (0);
   */
  public void setPipeline(int id) {
    pipeline.setDouble(id);
  }

  /**
   * Switches Limelight pipe to the driver pipe, which is thresholded to the equivalent to a webcam
   */
  public void setDriverPipe() {
    pipeline.setDouble(Constants.LimelightConstants.driverPipe);
  }

  /**
   * Switches Limelight pipe to the targeting pipe, which is thresholded for locating the hub
   */
  public void setTargetPipe() {
    pipeline.setDouble(Constants.LimelightConstants.targetPipe);
  }

  /**
   * Get the feed of the Limelight stream
   * @return the Limelight's camera stream
   */
  public HttpCamera getFeed() {
    return camera;
  }

  public PIDController getPIDController(){
    return limelightPID;
  }
}
