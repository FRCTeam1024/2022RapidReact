// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final UsbCamera intakeCamera;

  private final WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.IntakeConstants.intakeMotorID);
  private final CANSparkMax shifterMotor = new CANSparkMax(Constants.IntakeConstants.shifterMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final Solenoid intakeValve = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.intakeValve);

  /** Creates a new Intake. */
  public Intake() {
    intakeCamera = CameraServer.startAutomaticCapture();
    //intakeCamera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 160, 90, 5);
    //intakeCamera.setExposureManual(5);

    shifterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);
    shifterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1000);
    shifterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);

    intakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 1000);
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
  * Run intake forward to collect Cargo at speed defined by constant
  * */
  public void collect() {
    runIntake(IntakeConstants.kCollectSpeed);
    runShifter(IntakeConstants.kShifterSpeed);
  }

  /**
   * Run intake in reverse to eject/unjam Cargo at speed defined by constant
   */
  public void eject() {
    extend();
    runIntake(IntakeConstants.kEjectSpeed);
    runShifter(-IntakeConstants.kShifterSpeed);
  }

  /**
   * Retract the intake and stop rotation
   */
  public void stow() {
    runIntake(0);
    runShifter(0);
    retract();
  }

  /**
   * Retract and stop the intake (if not already) but keep running the 
   * shifter in order to feed cargo to the launcher
   */
  public void feedLauncher() {
    runIntake(0);
    runShifter(IntakeConstants.kShifterSpeed);
    retract();
  }

  /**
   * Extend the intake and run in collect mode
   */
  public void deploy() {
    extend();
    collect();
  }

  public UsbCamera getFeed() {
    return intakeCamera;
  }

  /**
   * This method reports on if the shifter is running
   * in the direction towards the launcher.
   * 
   * @return TRUE if shifter feeding balls towards the launcher
   */
  public boolean shifterRunning() {
    return shifterMotor.get() > 0; 
  }

  /**
   * Extend the intake, use the cylinder
   */
  private void extend() {
    intakeValve.set(true);  //May need to reverse boolean
  }

  /**
   * Retract the intake, use the cylinder
   */
  private void retract() {
    intakeValve.set(false); //May need to reverse boolean
    System.out.println("Stowing");
    
  }

  /**
   * Run intake to speed and direction sepecified.  This is the only
   * place the intake motor should be set directly.
   * 
   * @param pwr %power for intake motor (+ = collect)
   */
  private void runIntake(double pwr) {
      intakeMotor.set(pwr);
  }

  /**
   * Run the cargo shifter to speed and direction specified.  This is
   * the only place the shifter motor should be set.
   * 
   * @param pwr %power for the shifter motor (+ towards launcher)
   */
  public void runShifter(double pwr) {
    shifterMotor.set(pwr);
  }
}
