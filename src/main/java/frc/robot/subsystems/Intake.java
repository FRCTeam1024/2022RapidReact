// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  private final UsbCamera intakeCamera;

  /** Creates a new Intake. */
  public Intake() {
    intakeCamera = CameraServer.startAutomaticCapture();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Run intake to speed and direction sepecified
   * 
   * @param speed %power for intake motor (+ collect)
   */
  public void runIntake(double speed) {

  }

  /**
  * Run intake forward to collect Cargo
  * */
  public void intakeCollect() {

  }

  /**
   * Run intake in reverse to eject/unjam Cargo
   */
  public void intakeEject() {
    
  }

  /**
   * Extend the intake
   */
  public void extendIntake() {

  }

  /**
   * Retract the intake
   */
  public void retractIntake() {

  }

  /**
   * Retract the intake and stop rotation
   */
  public void stowIntake() {
    runIntake(0);
    retractIntake();
  }

  /**
   * Extend the intake and run in collect mode
   */
  public void deployIntake() {
    extendIntake();
    runIntake(IntakeConstants.kCollectSpeed);
  }

  // Alex: see line 14
  public UsbCamera getFeed() {
    return intakeCamera;
  }
}
